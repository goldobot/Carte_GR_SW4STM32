#include "goldobot/odrive/odrive_client.hpp"

#include "goldobot/core/message_exchange.hpp"

#include <cassert>

namespace goldobot {
constexpr uint16_t ODriveClient::c_axis_base[2];
constexpr uint16_t ODriveClient::c_endpoint_vel_estimate;
constexpr uint16_t ODriveClient::c_endpoint_axis_error;
constexpr uint16_t ODriveClient::c_endpoint_motor_error;
constexpr uint16_t ODriveClient::c_endpoint_input_vel;
constexpr uint16_t ODriveClient::c_endpoint_current_state;
constexpr uint16_t ODriveClient::c_endpoint_requested_state;
constexpr uint16_t ODriveClient::c_endpoint_control_mode;
constexpr uint32_t ODriveClient::c_odrive_consts_axis_state[3];
constexpr uint32_t ODriveClient::c_odrive_consts_control_mode;

const ODriveClient::Errors& ODriveClient::errors() const noexcept { return m_errors; }
const ODriveClient::Telemetry& ODriveClient::telemetry() const noexcept { return m_telemetry; }
const ODriveClient::AxisStates ODriveClient::axisStates() const noexcept { return m_axis_states; }

template <typename T>
ODriveClient::sequence_number_t ODriveClient::queueReadEndpoint(endpoint_id_t endpoint) {
  auto seq = m_seq;
  uint8_t buff[8];

  *reinterpret_cast<uint16_t*>(buff + 0) = seq | 0x4000;
  *reinterpret_cast<uint16_t*>(buff + 2) = endpoint | 0x8000;
  *reinterpret_cast<uint16_t*>(buff + 4) = sizeof(T);
  *reinterpret_cast<uint16_t*>(buff + 6) = c_odrive_key;
  m_seq = (m_seq + 1) & 0x1fff;
  if (m_exchange) {
    m_exchange->pushMessage(m_request_packet_message_type, buff, sizeof(buff));
  }
  return seq;
}

template <typename T>
ODriveClient::sequence_number_t ODriveClient::writeEndpoint(endpoint_id_t endpoint, const T& val,
                                                            bool ack_requested) {
  auto seq = m_seq;
  uint8_t buff[8 + sizeof(T)];

  *reinterpret_cast<uint16_t*>(buff + 0) = m_seq | 0x4000;
  *reinterpret_cast<uint16_t*>(buff + 2) = endpoint | (ack_requested ? 0x8000 : 0);
  *reinterpret_cast<uint16_t*>(buff + 4) = 0;
  *reinterpret_cast<T*>(buff + 6) = val;
  *reinterpret_cast<uint16_t*>(buff + 6 + sizeof(T)) = c_odrive_key;
  m_seq = (m_seq + 1) & 0x1fff;

  if (m_exchange) {
    m_exchange->pushMessage(m_request_packet_message_type, buff, sizeof(buff));
  }
  return seq;
}

void ODriveClient::setOutputExchange(MessageExchange* exchange, CommMessageType message_type) {
  m_exchange = exchange;
  m_request_packet_message_type = message_type;
}

void ODriveClient::setMotorsEnable(bool enable) {
  uint32_t axis_state = enable ? c_odrive_consts_axis_state[1] : c_odrive_consts_axis_state[0];

  // todo: manage timeouts and retransmissions

  // Set velocity control and enable or disable closed loop control
  for (int i = 0; i < 2; i++) {
    m_reqs[i].requested_state = axis_state;
    m_reqs[i].control_mode = c_odrive_consts_control_mode;

    writeEndpoint(c_endpoint_control_mode + c_axis_base[i], c_odrive_consts_control_mode);
    writeEndpoint(c_endpoint_requested_state + c_axis_base[i], axis_state);
  }
}

void ODriveClient::setVelocitySetPoint(int axis, float vel_setpoint, float current_feedforward) {
  assert(axis >= 0 && axis < 2);
  m_reqs[axis].input_vel = vel_setpoint;
  m_reqs[axis].input_torque = current_feedforward;
}

void ODriveClient::clearErrors() {
  for (int i = 0; i < 2; i++) {
    m_reqs[0].seq_clear_errors =
        writeEndpoint<uint32_t>(c_endpoint_clear_errors + c_axis_base[0], 0);
    m_flags_clear_errors = 0x80;
    // todo: manage timeouts and retransmissions
  }
}

bool ODriveClient::startMotorsCalibration() {
  // if (m_odrive_calibration_state != 0) {
  //  return false;
  //}
  writeEndpoint(c_endpoint_requested_state + c_axis_base[0], 2);
  writeEndpoint(c_endpoint_requested_state + c_axis_base[1], 2);

  // m_odrive_calibration_state = 1;
  // m_axis_current_state[0] = 0;
  return true;
}

void ODriveClient::doStep(uint32_t timestamp) {
  // send motor inputs
  if (m_cnt == m_cnt_next_set_velocity_setpoints) {
    for (int i = 0; i < 2; i++) {
      writeEndpoint<float>(c_endpoint_input_vel + c_axis_base[i], m_reqs[i].input_vel);
      writeEndpoint<float>(c_endpoint_input_torque + c_axis_base[i], m_reqs[i].input_torque);
    }
    m_cnt_next_set_velocity_setpoints += m_config.req_set_vel_setpoints_period;
  };

  // reqeust telemetry data
  if (m_cnt == m_cnt_next_request_telemetry) {
    for (int i = 0; i < 2; i++) {
      m_reqs[i].seq_pos_estimate =
          queueReadEndpoint<float>(c_endpoint_pos_estimate + c_axis_base[i]);
      m_reqs[i].seq_vel_estimate =
          queueReadEndpoint<float>(c_endpoint_vel_estimate + c_axis_base[i]);
      m_reqs[i].seq_current_iq_setpoint =
          queueReadEndpoint<float>(c_endpoint_current_iq_setpoint + c_axis_base[i]);
    }
    m_telemetry.timestamp = timestamp;
    m_cnt_next_request_telemetry += m_config.req_telemetry_period;
  }

  // request error flags
  if (m_cnt == m_cnt_next_request_errors) {
    for (int i = 0; i < 2; i++) {
      m_reqs[i].seq_axis_error = queueReadEndpoint<float>(c_endpoint_axis_error + c_axis_base[0]);
    }
    m_cnt_next_request_errors += m_config.req_errors_period;
  };

  // request axis states
  if (m_cnt == m_cnt_next_request_axis_states) {
    for (int i = 0; i < 2; i++) {
      m_reqs[i].seq_current_state =
          queueReadEndpoint<float>(c_endpoint_current_state + c_axis_base[i]);
    }
    m_cnt_next_request_axis_states += m_config.req_axis_states_period;
  };
  m_cnt++;
}

bool ODriveClient::processResponse(sequence_number_t seq, uint8_t* payload, size_t payload_size) {
  for (int i = 0; i < 2; i++) {
    // state
    if (seq == m_reqs[i].seq_current_state) {
      m_axis_states.axis[i] = *(uint32_t*)(payload);
      m_reqs[i].seq_current_state = 0;
    }

    if (seq == m_reqs[i].seq_requested_state) {
      m_reqs[i].seq_requested_state = 0;
    }

    // telemetry
    if (seq == m_reqs[i].seq_pos_estimate) {
      m_telemetry.axis[i].pos_estimate = *(float*)(payload);
      m_reqs[i].seq_pos_estimate = 0;
    }
    if (seq == m_reqs[i].seq_vel_estimate) {
      m_telemetry.axis[i].vel_estimate = *(float*)(payload);
      m_reqs[i].seq_vel_estimate = 0;
    }
    if (seq == m_reqs[i].seq_current_iq_setpoint) {
      m_telemetry.axis[i].current_iq_setpoint = *(float*)(payload);
      m_reqs[i].seq_current_iq_setpoint = 0;
    }

    // errors
    if (seq == m_reqs[i].seq_axis_error) {
      m_errors.axis[i].axis = *(uint32_t*)(payload);
      m_reqs[i].seq_axis_error = 0;
    }

    if (seq == m_reqs[i].seq_motor_error) {
      m_errors.axis[i].motor = *(uint32_t*)(payload);
      m_reqs[i].seq_motor_error = 0;
    }

    if (seq == m_reqs[i].seq_controller_error) {
      m_errors.axis[i].controller = *(uint32_t*)(payload);
      m_reqs[i].seq_controller_error = 0;
    }

    // clear errors
    if (seq == m_reqs[i].seq_clear_errors) {
      m_flags_clear_errors |= (1 << i);
      m_reqs[i].seq_clear_errors = 0;
      // all acks have been received for the clear errors command, reset flags to 0
      if (m_flags_clear_errors & 0x83) {
        m_flags_clear_errors = 0;
      }
    }
  }

  // axis current state
  // if (m_odrive_calibration_state == 1 && m_axis_current_state[0] == 4) {
  // axis0 calibration started
  //  m_odrive_calibration_state = 2;
  //  }
  /*
  if (seq == m_seq_axis_current_state[0]) {
    m_axis_current_state[0] = *(uint32_t*)(payload);
    if (m_odrive_calibration_state == 1 && m_axis_current_state[0] == 4) {
      // axis0 calibration started
      m_odrive_calibration_state = 2;
    }
    if (m_odrive_calibration_state == 2 && m_axis_current_state[0] == 1) {
      // axis0 calibration finished
      m_odrive_calibration_state = 3;
      // start calibration of axis1
      writeEndpoint(c_endpoint_requested_state[1], c_odrive_consts_axis_state[2]);
      m_axis_current_state[1] = 0;
    }
  }*/

  /*
  if (seq == m_seq_axis_current_state[1]) {
    m_axis_current_state[1] = *(uint32_t*)(payload);
    if (m_odrive_calibration_state == 3 && m_axis_current_state[1] == 4) {
      // axis1 calibration started
      m_odrive_calibration_state = 4;
    }
    if (m_odrive_calibration_state == 4 && m_axis_current_state[1] == 1) {
      // axis1 calibration finished
      m_odrive_calibration_state = 0;
    }
  }
  */

  return true;
}

}  // namespace goldobot
