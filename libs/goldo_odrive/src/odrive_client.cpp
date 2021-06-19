#include "goldobot/odrive/odrive_client.hpp"

#include "goldobot/core/message_exchange.hpp"

#include <cassert>

namespace goldobot {
constexpr uint16_t ODriveClient::c_axis_base[2];
constexpr uint32_t ODriveClient::c_odrive_consts_axis_state[3];
constexpr uint32_t ODriveClient::c_odrive_consts_control_mode;

const uint16_t ODriveClient::c_endpoints[25] = {
    2,    // current state
    3,    // requested state
    137,  // control mode
    124,  // input vel
    125,  // input torque
    126,  // torque limit
    172,  // pos estimate
    178,  // vel estimate
    78,   // current iq setpoint
    0,    // axis errors
    61,   // motor errors
    122,  // controller errors
    165,  // encoder errors
    203,  // sensorless estimator errors
    140,  // vel_gain
    141,  // vel_integrator_gain
    166,  // encoder is_ready
    62,   // motor is_armed
    63,   // motor is_calibrated
    47,   // fet thermistor temperature
    52,   // motor thermistor temperature
    1,    // bus voltage
    2,    // bus current
    228,  // clear errors
    346   // reboot
};

const std::array<ODriveClient::AxisErrorState, 2>& ODriveClient::errors() const noexcept {
  return m_errors;
}
const ODriveClient::Telemetry& ODriveClient::telemetry() const noexcept { return m_telemetry; }
const std::array<ODriveClient::AxisState, 2>& ODriveClient::axisStates() const noexcept {
  return m_axis_states;
}
const std::array<ODriveClient::AxisCalibrationState, 2>& ODriveClient::axisCalibrationStates()
    const noexcept {
  return m_axis_calibration_states;
};

template <typename T>
ODriveClient::sequence_number_t ODriveClient::queueReadEndpoint(endpoint_id_t endpoint,
                                                                uint8_t req_id) {
  auto seq = m_seq;
  m_seq = (m_seq + 1);
  seq = (seq & 0x1f) | 0x20 | (req_id << 6) | 0x4000;

  uint8_t buff[8];

  *reinterpret_cast<uint16_t*>(buff + 0) = seq;
  *reinterpret_cast<uint16_t*>(buff + 2) = endpoint | 0x8000;
  *reinterpret_cast<uint16_t*>(buff + 4) = sizeof(T);
  *reinterpret_cast<uint16_t*>(buff + 6) = c_odrive_key;

  if (m_exchange) {
    m_exchange->pushMessage(m_request_packet_message_type, buff, sizeof(buff));
  }
  return seq;
}

template <typename T>
ODriveClient::sequence_number_t ODriveClient::writeEndpoint(endpoint_id_t endpoint, const T& val,
                                                            uint8_t req_id, bool ack_requested) {
  auto seq = m_seq;
  m_seq = (m_seq + 1);
  seq = (seq & 0x1f) | 0x20 | (req_id << 6) | 0x4000;
  uint8_t buff[8 + sizeof(T)];

  *reinterpret_cast<uint16_t*>(buff + 0) = seq;
  *reinterpret_cast<uint16_t*>(buff + 2) = endpoint | (ack_requested ? 0x8000 : 0);
  *reinterpret_cast<uint16_t*>(buff + 4) = 0;
  *reinterpret_cast<T*>(buff + 6) = val;
  *reinterpret_cast<uint16_t*>(buff + 6 + sizeof(T)) = c_odrive_key;

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
    m_axis_requests[i].requested_state = axis_state;
    m_axis_requests[i].control_mode = c_odrive_consts_control_mode;
    m_axis_requests[i].write_requested_flags |= 0x3;
  }
}

void ODriveClient::setVelocitySetPoint(int axis, float vel_setpoint, float current_feedforward) {
  assert(axis >= 0 && axis < 2);
  m_axis_requests[axis].input_vel = vel_setpoint;
  m_axis_requests[axis].input_torque = current_feedforward;
}

void ODriveClient::clearErrors() {
  /*for (int i = 0; i < 2; i++) {
    m_reqs[0].seq_clear_errors =
        writeEndpoint<uint32_t>(c_endpoint_clear_errors + c_axis_base[0], 0);
    m_flags_clear_errors = 0x80;*/
  // todo: manage timeouts and retransmissions
  // }
}

bool ODriveClient::startMotorsCalibration() {
  // if (m_odrive_calibration_state != 0) {
  //  return false;
  //}
  // writeEndpoint(c_endpoint_requested_state + c_axis_base[0], 2);
  // writeEndpoint(c_endpoint_requested_state + c_axis_base[1], 2);

  // m_odrive_calibration_state = 1;
  // m_axis_current_state[0] = 0;
  return true;
}

void ODriveClient::doStep(uint32_t timestamp) {
  int reqs_left = 3;  // maximum number of requests sent per cycle
  // one request = 8 bytes + 5 bytes of uart protocol overhead
  // at 500kbps, the uart can send 50 bytes per 1 ms cycle
  // this allows 3 requests per cycle


  if (!m_is_synchronized) {
    while (m_synchronize_idx < 21 && reqs_left > 0) {
      if (m_axis_requests[0].read_requested_flags & (1 << m_synchronize_idx)) {
        queueReadRequest(0, static_cast<AxisRequestId>(m_synchronize_idx), timestamp);
        reqs_left--;
      }
      if (m_axis_requests[1].read_requested_flags & (1 << m_synchronize_idx)) {
        queueReadRequest(1, static_cast<AxisRequestId>(m_synchronize_idx), timestamp);
        reqs_left--;
      }
      m_synchronize_timestamp = timestamp;
      m_synchronize_idx++;
    }
    if (m_synchronize_idx == 21) {
      if (timestamp - m_synchronize_timestamp > 100) {
        m_axis_requests[0].read_requested_flags = m_axis_requests[0].read_pending_flags;
        m_axis_requests[1].read_requested_flags = m_axis_requests[1].read_pending_flags;
        m_synchronize_idx = 0;
      }

      if ((m_axis_requests[0].read_pending_flags & 0x1fffff) == 0 &&
          (m_axis_requests[1].read_pending_flags & 0x1fffff) == 0) {
        m_is_synchronized = true;
        m_synchronize_idx = 0;
      }
    }
  }

  if (!m_is_synchronized) {
    return;
  }

  // read axis state
  if (timestamp > m_next_read_states_timestamp) {
    const uint32_t read_flags = 0x1c0;
    m_next_read_states_timestamp =
        std::max(m_next_read_states_timestamp + m_config.req_axis_states_period, timestamp);
    m_axis_requests[0].read_requested_flags |= read_flags;
    m_axis_requests[1].read_requested_flags |= read_flags;
  }

  // read error state
  if (timestamp > m_next_read_errors_timestamp) {
    const uint32_t read_flags = 0x3e00;
    // from axis_error to sensorless_estimator_error
    m_next_read_errors_timestamp =
        std::max(m_next_read_errors_timestamp + m_config.req_telemetry_period, timestamp);
    m_axis_requests[0].read_requested_flags |= read_flags;
    m_axis_requests[1].read_requested_flags |= read_flags;
  }

  // send inputs
  if (timestamp > m_next_write_inputs_timestamp) {
    const uint32_t write_flags = 0x38;
    // input_vel, input_torque, torque_limit
    m_next_write_inputs_timestamp =
        std::max(m_next_write_inputs_timestamp + m_config.req_set_vel_setpoints_period, timestamp);
    m_axis_requests[0].write_requested_flags |= write_flags;
    m_axis_requests[1].write_requested_flags |= write_flags;
  }

  // read telemetry data
  if (timestamp > m_next_read_telemetry_timestamp) {
    const uint32_t write_flags = 0x1c0;
    // pos_estimate, vel_estimate, current_iq_setpoint
    m_next_read_telemetry_timestamp =
        std::max(m_next_read_telemetry_timestamp + m_config.req_telemetry_period, timestamp);
    m_axis_requests[0].read_requested_flags |= write_flags;
    m_axis_requests[1].read_requested_flags |= write_flags;
  }

  while (m_req_idx < 21 && reqs_left > 0) {
    for (int axis = 0; axis < 2 && reqs_left > 0; axis++) {
      const auto& reqs = m_axis_requests[axis];
      if (reqs.write_requested_flags & (1 << m_req_idx)) {
        writeRequest(axis, static_cast<AxisRequestId>(m_req_idx), timestamp);
        reqs_left--;
      }
      if (reqs.read_requested_flags & (1 << m_req_idx)) {
        queueReadRequest(0, static_cast<AxisRequestId>(m_req_idx), timestamp);
        reqs_left--;
      }
    }
    m_synchronize_timestamp = timestamp;
    m_req_idx++;
  }
  if (m_req_idx == 21) {
    if (timestamp - m_synchronize_timestamp > 30) {
      m_req_idx = 0;
    }
  }
}

bool ODriveClient::processResponse(uint32_t timestamp, sequence_number_t seq, uint8_t* payload,
                                   size_t payload_size) {
  uint8_t req_id = (seq >> 6) & 0xff;
  int axis = req_id >> 6;
  req_id = req_id & 0x3f;
  seq = seq & 0x3f;

  if (axis == 1 || axis == 2) {
    axis = axis - 1;
    uint8_t latency{0};
    if (payload_size > 0) {
      if (m_axis_requests[axis].seq_numbers[req_id] == seq) {
        m_axis_requests[axis].seq_numbers[req_id] = 0;
        m_axis_requests[axis].read_pending_flags &=
            0xffffffff - (1 << static_cast<uint8_t>(req_id));
        latency = (uint8_t)timestamp - m_axis_requests[axis].req_timestamps[req_id];
        processReadResponse(axis, (AxisRequestId)req_id, payload);
      }
    } else {
      if (m_axis_requests[axis].write_seq_numbers[req_id] == seq) {
        m_axis_requests[axis].write_seq_numbers[req_id] = 0;
        m_axis_requests[axis].write_pending_flags &=
            0xffffffff - (1 << static_cast<uint8_t>(req_id));
        latency = (uint8_t)timestamp - m_axis_requests[axis].write_req_timestamps[req_id];
      }
      m_statistics.max_latency = std::max<uint16_t>(m_statistics.max_latency, latency);
    }
  }

  return true;
}

void ODriveClient::queueReadRequest(int axis, AxisRequestId req_id, uint32_t timestamp) {
  uint8_t req_id_bin = static_cast<uint8_t>(req_id) | ((axis + 1) << 6);
  auto endpoint_id = c_endpoints[static_cast<uint8_t>(req_id)] + c_axis_base[axis];

  uint16_t seq{0};

  switch (req_id) {
    case AxisRequestId::CurrentState:
      seq = queueReadEndpoint<int32_t>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::RequestedState:
      seq = queueReadEndpoint<int32_t>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::ControlMode:
      seq = queueReadEndpoint<int32_t>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::InputVel:
      seq = queueReadEndpoint<float>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::InputTorque:
      seq = queueReadEndpoint<float>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::TorqueLimit:
      seq = queueReadEndpoint<float>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::PosEstimate:
      seq = queueReadEndpoint<float>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::VelEstimate:
      seq = queueReadEndpoint<float>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::CurrentIqSetpoint:
      seq = queueReadEndpoint<int32_t>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::AxisError:
      seq = queueReadEndpoint<int32_t>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::MotorError:
      seq = queueReadEndpoint<int32_t>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::ControllerError:
      seq = queueReadEndpoint<int32_t>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::EncoderError:
      seq = queueReadEndpoint<int32_t>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::SensorlessEstimatorError:
      seq = queueReadEndpoint<int32_t>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::VelGain:
      seq = queueReadEndpoint<float>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::VelIntegratorGain:
      seq = queueReadEndpoint<float>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::EncoderIsReady:
      seq = queueReadEndpoint<bool>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::MotorIsArmed:
      seq = queueReadEndpoint<int32_t>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::MotorIsCalibrated:
      seq = queueReadEndpoint<bool>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::FetThermistorTemperature:
      seq = queueReadEndpoint<float>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::MotorThermistorTemperature:
      seq = queueReadEndpoint<float>(endpoint_id, req_id_bin);
      break;
    default:
      break;
  }
  m_axis_requests[axis].read_requested_flags &= 0xffffffff - (1 << static_cast<uint8_t>(req_id));
  m_axis_requests[axis].read_pending_flags |= (1 << static_cast<uint8_t>(req_id));
  m_axis_requests[axis].seq_numbers[static_cast<uint8_t>(req_id)] = (seq & 0x3f);
  m_axis_requests[axis].req_timestamps[static_cast<uint8_t>(req_id)] =
      static_cast<uint8_t>(timestamp & 0xff);
}

void ODriveClient::writeRequest(int axis, AxisRequestId req_id, uint32_t timestamp) {
  uint8_t req_id_bin = static_cast<uint8_t>(req_id) | ((axis + 1) << 6);
  auto endpoint_id = c_endpoints[static_cast<uint8_t>(req_id)] + c_axis_base[axis];

  uint16_t seq{0};

  switch (req_id) {
    case AxisRequestId::CurrentState:
      break;
    case AxisRequestId::RequestedState:
      seq = writeEndpoint(endpoint_id, m_axis_requests[axis].requested_state, req_id_bin, true);
      break;
    case AxisRequestId::ControlMode:
      seq = writeEndpoint(endpoint_id, m_axis_requests[axis].control_mode, req_id_bin, true);
      break;
    case AxisRequestId::InputVel:
      seq = writeEndpoint(endpoint_id, m_axis_requests[axis].input_vel, req_id_bin, true);
      break;
    case AxisRequestId::InputTorque:
      seq = writeEndpoint(endpoint_id, m_axis_requests[axis].input_torque, req_id_bin, true);
      break;
    case AxisRequestId::TorqueLimit:
      seq = writeEndpoint(endpoint_id, m_axis_requests[axis].torque_lim, req_id_bin, true);
      break;
    case AxisRequestId::PosEstimate:
      seq = queueReadEndpoint<float>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::VelEstimate:
      seq = queueReadEndpoint<float>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::CurrentIqSetpoint:
      seq = queueReadEndpoint<int32_t>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::AxisError:
      seq = queueReadEndpoint<int32_t>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::MotorError:
      seq = queueReadEndpoint<int32_t>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::ControllerError:
      seq = queueReadEndpoint<int32_t>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::EncoderError:
      seq = queueReadEndpoint<int32_t>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::SensorlessEstimatorError:
      seq = queueReadEndpoint<int32_t>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::VelGain:
      seq = queueReadEndpoint<float>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::VelIntegratorGain:
      seq = queueReadEndpoint<float>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::EncoderIsReady:
      seq = queueReadEndpoint<bool>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::MotorIsArmed:
      seq = queueReadEndpoint<int32_t>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::MotorIsCalibrated:
      seq = queueReadEndpoint<bool>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::FetThermistorTemperature:
      seq = queueReadEndpoint<float>(endpoint_id, req_id_bin);
      break;
    case AxisRequestId::MotorThermistorTemperature:
      seq = queueReadEndpoint<float>(endpoint_id, req_id_bin);
      break;
    default:
      break;
  }

  m_axis_requests[axis].write_requested_flags &= 0xffffffff - (1 << static_cast<uint8_t>(req_id));
  m_axis_requests[axis].write_pending_flags |= (1 << static_cast<uint8_t>(req_id));
  m_axis_requests[axis].write_seq_numbers[static_cast<uint8_t>(req_id)] = seq;
  m_axis_requests[axis].write_req_timestamps[static_cast<uint8_t>(req_id)] =
      static_cast<uint8_t>(timestamp & 0xff);
}

void ODriveClient::processReadResponse(int axis, AxisRequestId req_id, uint8_t* payload) {
  switch (req_id) {
    case AxisRequestId::CurrentState:
      m_axis_states[axis].current_state = *(int32_t*)payload;
      break;
    case AxisRequestId::RequestedState:
      m_axis_states[axis].requested_state = *(int32_t*)payload;
      break;
    case AxisRequestId::ControlMode:
      m_axis_states[axis].control_mode = *(int32_t*)payload;
      break;
    case AxisRequestId::InputVel:

      break;
    case AxisRequestId::InputTorque:

      break;
    case AxisRequestId::TorqueLimit:

      break;
    case AxisRequestId::PosEstimate:
      m_telemetry.axis[axis].pos_estimate = *(float*)payload;
      break;
    case AxisRequestId::VelEstimate:
      m_telemetry.axis[axis].vel_estimate = *(float*)payload;
      break;
    case AxisRequestId::CurrentIqSetpoint:
      m_telemetry.axis[axis].current_iq_setpoint = *(float*)payload;
      break;
    case AxisRequestId::AxisError:
      m_errors[axis].axis = *(int32_t*)payload;
      break;
    case AxisRequestId::MotorError:
      m_errors[axis].motor = *(int32_t*)payload;
      break;
    case AxisRequestId::ControllerError:
      m_errors[axis].controller = *(int32_t*)payload;
      break;
    case AxisRequestId::EncoderError:
      m_errors[axis].encoder = *(int32_t*)payload;
      break;
    case AxisRequestId::SensorlessEstimatorError:
      m_errors[axis].sensorless_estimator = *(int32_t*)payload;
      break;
    case AxisRequestId::VelGain:

      break;
    case AxisRequestId::VelIntegratorGain:

      break;
    case AxisRequestId::EncoderIsReady:
      m_axis_calibration_states[axis].encoder_is_ready = *(bool*)payload;
      break;
    case AxisRequestId::MotorIsArmed:

      break;
    case AxisRequestId::MotorIsCalibrated:

      break;
    case AxisRequestId::FetThermistorTemperature:

      break;
    case AxisRequestId::MotorThermistorTemperature:

      break;
    default:
      break;
  }
}

}  // namespace goldobot
