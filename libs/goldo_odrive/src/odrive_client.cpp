#include "goldobot/odrive/odrive_client.hpp"

#include "goldobot/core/message_exchange.hpp"

#include <cassert>

namespace goldobot {

constexpr uint16_t ODriveClient::c_endpoint_vel_estimate[2];
constexpr uint16_t ODriveClient::c_endpoint_axis_error[2];
constexpr uint16_t ODriveClient::c_endpoint_motor_error[2];
constexpr uint16_t ODriveClient::c_endpoint_input_vel[2];
constexpr uint16_t ODriveClient::c_endpoint_current_state[2];
constexpr uint16_t ODriveClient::c_endpoint_requested_state[2];
constexpr uint16_t ODriveClient::c_endpoint_control_mode[2];
constexpr uint32_t ODriveClient::c_odrive_consts_axis_state[3];
constexpr uint32_t ODriveClient::c_odrive_consts_control_mode;

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
ODriveClient::sequence_number_t ODriveClient::writeEndpoint(endpoint_id_t endpoint, const T& val) {
  auto seq = m_seq;
  uint8_t buff[8 + sizeof(T)];

  *reinterpret_cast<uint16_t*>(buff + 0) = m_seq | 0x4000;
  *reinterpret_cast<uint16_t*>(buff + 2) = endpoint;
  *reinterpret_cast<uint16_t*>(buff + 4) = 0;
  *reinterpret_cast<T*>(buff + 6) = val;
  *reinterpret_cast<uint16_t*>(buff + 6 + sizeof(T)) = c_odrive_key;
  m_seq = (m_seq + 1) & 0x1fff;

  if (m_exchange) {
    m_exchange->pushMessage(m_request_packet_message_type, buff, sizeof(buff));
  }
  return seq;
}

float ODriveClient::axisVelEstimate(int axis) const noexcept { return m_axis_vel_estimate[axis]; }
uint32_t ODriveClient::axisCurrentState(int axis) const noexcept {
  return m_axis_current_state[axis];
}
uint32_t ODriveClient::axisError(int axis) const noexcept { return m_axis_error[axis]; }
uint32_t ODriveClient::motorError(int axis) const noexcept { return m_motor_error[axis]; }

void ODriveClient::setOutputExchange(MessageExchange* exchange, CommMessageType message_type) {
  m_exchange = exchange;
  m_request_packet_message_type = message_type;
}

void ODriveClient::setMotorsEnable(bool enable) {
  // Set velocity control
  writeEndpoint(c_endpoint_control_mode[0], c_odrive_consts_control_mode);
  writeEndpoint(c_endpoint_control_mode[1], c_odrive_consts_control_mode);

  // Enable or disable closed loop control
  uint32_t axis_state = enable ? c_odrive_consts_axis_state[1] : c_odrive_consts_axis_state[0];
  writeEndpoint(c_endpoint_requested_state[0], axis_state);
  writeEndpoint(c_endpoint_requested_state[1], axis_state);
}

void ODriveClient::setVelocitySetPoint(int axis, float vel_setpoint, float current_feedforward,
                                       bool immediate) {
  assert(axis >= 0 && axis < 2);
  if (immediate || m_cnt_next_set_velocity_setpoints == m_cnt) {
    writeEndpoint(c_endpoint_input_vel[axis], vel_setpoint);
    writeEndpoint(c_endpoint_input_vel[axis] + 1, current_feedforward);
    m_cnt_next_set_velocity_setpoints = m_cnt + 10;
  }
}

void ODriveClient::clearErrors() {
  writeEndpoint<uint32_t>(c_endpoint_axis_error[0], 0);
  writeEndpoint<uint32_t>(c_endpoint_axis_error[1], 0);
  writeEndpoint<uint32_t>(c_endpoint_motor_error[0], 0);
  writeEndpoint<uint32_t>(c_endpoint_motor_error[1], 0);
}

bool ODriveClient::startMotorsCalibration() {
  if (m_odrive_calibration_state != 0) {
    return false;
  }
  writeEndpoint(c_endpoint_requested_state[0], c_odrive_consts_axis_state[2]);
  m_odrive_calibration_state = 1;
  m_axis_current_state[0] = 0;
  return true;
}

void ODriveClient::doStep() {
  if (m_cnt == m_cnt_next_request_vel_estimates) {
    m_seq_vel_estimate[0] = queueReadEndpoint<float>(c_endpoint_vel_estimate[0]);
    m_seq_vel_estimate[1] = queueReadEndpoint<float>(c_endpoint_vel_estimate[1]);
    m_cnt_next_request_vel_estimates += 10;
  }

  if (m_cnt == m_cnt_next_request_axis_errors) {
    m_seq_axis_error[0] = queueReadEndpoint<float>(c_endpoint_axis_error[0]);
    m_seq_axis_error[1] = queueReadEndpoint<float>(c_endpoint_axis_error[1]);
    m_cnt_next_request_axis_errors += 20;
  };

  if (m_cnt == m_cnt_next_request_motor_errors) {
    m_seq_motor_error[0] = queueReadEndpoint<float>(c_endpoint_motor_error[0]);
    m_seq_motor_error[1] = queueReadEndpoint<float>(c_endpoint_motor_error[1]);
    m_cnt_next_request_motor_errors += 20;
  };

  if (m_cnt == m_cnt_next_request_axis_states) {
    m_seq_axis_current_state[0] = queueReadEndpoint<float>(c_endpoint_current_state[0]);
    m_seq_axis_current_state[1] = queueReadEndpoint<float>(c_endpoint_current_state[1]);
    m_cnt_next_request_axis_states += 20;
  };
  m_cnt++;
}

/*
  if (m_odrive_cnt == 0) {
    uint8_t buff[26];
    *reinterpret_cast<float*>(buff + 0) = m_odrive_axis0_vel_estimate;
    *reinterpret_cast<float*>(buff + 4) = m_odrive_axis1_vel_estimate;
    *reinterpret_cast<uint32_t*>(buff + 8) = m_odrive_axis0_error;
    *reinterpret_cast<uint32_t*>(buff + 12) = m_odrive_axis1_error;
    *reinterpret_cast<uint32_t*>(buff + 16) = m_odrive_axis0_motor_error;
    *reinterpret_cast<uint32_t*>(buff + 20) = m_odrive_axis1_motor_error;
    *reinterpret_cast<uint8_t*>(buff + 24) = m_odrive_axis0_current_state;
    *reinterpret_cast<uint8_t*>(buff + 25) = m_odrive_axis1_current_state;

    Robot::instance().mainExchangeOut().pushMessage(CommMessageType::ODriveTelemetry,
                                                    (unsigned char*)buff, 26);*/

bool ODriveClient::processResponse(sequence_number_t seq, uint8_t* payload, size_t payload_size) {
  // velocity
  if (seq == m_seq_vel_estimate[0]) {
    m_axis_vel_estimate[0] = *(float*)(payload);
  }
  if (seq == m_seq_vel_estimate[1]) {
    m_axis_vel_estimate[0] = *(float*)(payload);
  }
  // axis current state
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
  }

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

  // axis error
  if (seq == m_seq_axis_error[0]) {
    m_axis_error[0] = *(uint32_t*)(payload);
  }
  if (seq == m_seq_axis_error[1]) {
    m_axis_error[1] = *(uint32_t*)(payload);
  }
  // axis motor error
  if (seq == m_seq_motor_error[0]) {
    m_motor_error[0] = *(uint32_t*)(payload);
  }
  if (seq == m_seq_motor_error[1]) {
    m_motor_error[1] = *(uint32_t*)(payload);
  }
  return true;
}

}  // namespace goldobot
