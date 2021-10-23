#include "goldobot/odrive/odrive_client.hpp"

#include "goldobot/core/message_exchange.hpp"

#include <cassert>
#include <cstring>

namespace goldobot {
constexpr uint16_t ODriveClient::c_axis_base[2];
constexpr uint32_t ODriveClient::c_odrive_consts_axis_state[3];
constexpr uint32_t ODriveClient::c_odrive_consts_control_mode;

const uint16_t ODriveClient::c_endpoints[22] = {
    2,    // current state
    3,    // requested state
    137,  // control mode
    124,  // input vel
    125,  // input torque
    111,  // torque limit
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
    228   // clear errors
};

const uint16_t ODriveClient::c_odrv_endpoints[4] = {
    1,   // bus voltage
    2,   // bus current
    14,  // uptime
    546  // reboot
};

ODriveClient::ODriveClient() {
  requestSynchronization();
  memset(m_telemetry_ack, 0, sizeof(m_telemetry_ack));
}

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
  return (seq & 0x1f);
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
  return (seq & 0x1f);
}

void ODriveClient::setOutputExchange(MessageExchange* exchange, CommMessageType message_type) {
  m_exchange = exchange;
  m_request_packet_message_type = message_type;
}

ODriveClient::Statistics ODriveClient::statistics() {
  m_statistics.uptime = m_odrv_requests.uptime;
  m_statistics.is_synchronized = m_is_synchronized;

  Statistics retval = m_statistics;
  m_statistics = Statistics();
  return retval;
}

void ODriveClient::setMotorsEnable(bool enable) {
  uint32_t axis_state = enable ? c_odrive_consts_axis_state[1] : c_odrive_consts_axis_state[0];

  // todo: manage timeouts and retransmissions

  // Set velocity control and enable or disable closed loop control
  for (int i = 0; i < 2; i++) {
    m_axis_requests[i].requested_state = axis_state;
    m_axis_requests[i].control_mode = c_odrive_consts_control_mode;
    m_axis_requests[i].requestWrite(AxisRequestId::RequestedState);
    m_axis_requests[i].requestWrite(AxisRequestId::ControlMode);
  }
}

void ODriveClient::setVelocitySetPoint(int axis, float vel_setpoint, float current_feedforward) {
  assert(axis >= 0 && axis < 2);
  m_axis_requests[axis].input_vel = vel_setpoint;
  m_axis_requests[axis].input_torque = current_feedforward;
}

void ODriveClient::setTorqueLimit(int axis, float torque_lim) {
  assert(axis >= 0 && axis < 2);
  m_axis_requests[axis].torque_lim = torque_lim;
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
  m_current_timestamp = timestamp;
  updateEndpoints(timestamp);

  if (!m_is_synchronized) {
    checkSynchronization();
    return;
  }

  // check latency on uptime
  if (timestamp - m_last_uptime_ts > m_config.req_global_data_period * 2) {
    requestSynchronization();
  }

  // global state
  if (timestamp > m_next_read_global_state_timestamp) {
    m_next_read_global_state_timestamp =
        std::max(m_next_read_global_state_timestamp + m_config.req_global_data_period, timestamp);
    auto& reqs = m_odrv_requests;
    for (unsigned i = 0; i < 3; i++) {
      reqs.requestRead(static_cast<ODrvRequestId>(i));
    }
  }

  // axis states
  if (timestamp > m_next_read_states_timestamp) {
    for (unsigned axis = 0; axis < 2; axis++) {
      auto& reqs = m_axis_requests[axis];
      reqs.requestRead(AxisRequestId::CurrentState);
      reqs.requestRead(AxisRequestId::RequestedState);
      reqs.requestRead(AxisRequestId::ControlMode);
    }
    m_next_read_states_timestamp =
        std::max(m_next_read_states_timestamp + m_config.req_axis_states_period, timestamp);
  }

  // axis errors
  if (timestamp > m_next_read_errors_timestamp) {
    for (unsigned axis = 0; axis < 2; axis++) {
      auto& reqs = m_axis_requests[axis];
      reqs.requestRead(AxisRequestId::AxisError);
      reqs.requestRead(AxisRequestId::MotorError);
      reqs.requestRead(AxisRequestId::ControllerError);
      reqs.requestRead(AxisRequestId::EncoderError);
      reqs.requestRead(AxisRequestId::SensorlessEstimatorError);
    }
    m_next_read_errors_timestamp =
        std::max(m_next_read_errors_timestamp + m_config.req_errors_period, timestamp);
  }

  // axis telemetry
  if (timestamp > m_next_read_telemetry_timestamp) {
    for (unsigned axis = 0; axis < 2; axis++) {
      auto& reqs = m_axis_requests[axis];
      reqs.requestRead(AxisRequestId::PosEstimate);
      reqs.requestRead(AxisRequestId::VelEstimate);
      reqs.requestRead(AxisRequestId::CurrentIqSetpoint);
    }
    m_next_read_telemetry_timestamp =
        std::max(m_next_read_telemetry_timestamp + m_config.req_telemetry_period, timestamp);
  }

  if (timestamp > m_next_write_inputs_timestamp) {
    for (unsigned axis = 0; axis < 2; axis++) {
      auto& reqs = m_axis_requests[axis];
      reqs.requestWrite(AxisRequestId::InputVel);
      reqs.requestWrite(AxisRequestId::InputTorque);
      reqs.requestWrite(AxisRequestId::TorqueLimit);
    }
    m_next_write_inputs_timestamp =
        std::max(m_next_write_inputs_timestamp + m_config.req_set_vel_setpoints_period, timestamp);
  }
}

void ODriveClient::requestSynchronization() {
  m_is_synchronized = false;
  m_req_idx = 0;
  for (unsigned axis = 0; axis < 2; axis++) {
    auto& reqs = m_axis_requests[axis];
    for (unsigned i = 0; i < 21; i++) {
      reqs.setEndpointState(static_cast<AxisRequestId>(i), EndpointState::ReadRequested);
    }
  }
  auto& reqs = m_odrv_requests;
  reqs.uptime = 0;
  for (unsigned i = 0; i < 3; i++) {
    reqs.setEndpointState(static_cast<ODrvRequestId>(i), EndpointState::ReadRequested);
  }
}

void ODriveClient::checkSynchronization() {
  bool is_synchronized = true;
  for (unsigned axis = 0; axis < 2; axis++) {
    auto& reqs = m_axis_requests[axis];
    for (unsigned i = 0; i < 21; i++) {
      if (reqs.endpointState(static_cast<AxisRequestId>(i)) != EndpointState::Idle) {
        is_synchronized = false;
      }
    }
  }
  auto& reqs = m_odrv_requests;
  for (unsigned i = 0; i < 3; i++) {
    if (reqs.endpointState(static_cast<ODrvRequestId>(i)) != EndpointState::Idle) {
      is_synchronized = false;
    }
  }
  m_is_synchronized = is_synchronized;
}

void ODriveClient::updateAxisEndpoint(int axis, int req_idx, uint32_t timestamp, int& reqs_left) {
  auto& reqs = m_axis_requests[axis];
  auto req_id = static_cast<AxisRequestId>(req_idx);
  auto endpoint_state = reqs.endpointState(req_id);
  switch (endpoint_state) {
    case EndpointState::Idle:
      break;
    case EndpointState::ReadRequested:
      queueReadRequest(axis, req_id, timestamp);
      reqs_left--;
      break;
    case EndpointState::ReadPending: {
      uint8_t latency = (uint8_t)timestamp - reqs.req_timestamps[req_idx];
      if (latency > 30) {
        queueReadRequest(axis, req_id, timestamp);
        reqs_left--;
      }
    } break;
    case EndpointState::WriteRequested:
      writeRequest(axis, req_id, timestamp);
      reqs_left--;
      break;
    case EndpointState::WritePending: {
      uint8_t latency = (uint8_t)timestamp - reqs.req_timestamps[req_idx];
      if (latency > 30) {
        writeRequest(axis, req_id, timestamp);
        reqs_left--;
      }
    } break;
    default:
      break;
  }
}
void ODriveClient::updateEndpoints(uint32_t timestamp) {
  int reqs_left = 3;  // maximum number of requests sent per cycle
                      // one request = 8 bytes + 5 bytes of uart protocol overhead
                      // at 500kbps, the uart can send 50 bytes per 1 ms cycle
                      // this allows 3 requests per cycle

  // priority to requested_state, then inputs and telemetry
  for (int req_idx = 1; req_idx < 10 && reqs_left > 0; req_idx++) {
    for (int axis = 0; axis < 2 && reqs_left > 0; axis++) {
      updateAxisEndpoint(axis, req_idx, timestamp, reqs_left);
    }
  }

  while (m_req_idx < 22 && reqs_left > 0) {
    for (int axis = 0; axis < 2 && reqs_left > 0; axis++) {
      updateAxisEndpoint(axis, m_req_idx, timestamp, reqs_left);
    }
    m_req_idx++;
  }
  while (m_req_idx < 26 && reqs_left > 0) {
    auto& reqs = m_odrv_requests;
    auto req_id = static_cast<ODrvRequestId>(m_req_idx - 22);
    auto endpoint_state = reqs.endpointState(req_id);

    switch (endpoint_state) {
      case EndpointState::Idle:
        break;
      case EndpointState::ReadRequested:
        queueReadRequest(req_id, timestamp);
        reqs_left--;
        break;
      case EndpointState::ReadPending: {
        uint8_t latency = (uint8_t)timestamp - reqs.req_timestamps[m_req_idx - 22];
        if (latency > 30) {
          m_statistics.timeout_errors++;
          queueReadRequest(req_id, timestamp);
          reqs_left--;
        }
      } break;
      default:
        break;
    }
    m_req_idx++;
  }

  if (m_req_idx == 26) {
    m_req_idx = 0;
  }
}

bool ODriveClient::processResponse(uint32_t timestamp, sequence_number_t seq, uint8_t* payload,
                                   size_t payload_size) {
  uint8_t req_id = (seq >> 6) & 0xff;
  int axis = req_id >> 6;
  req_id = req_id & 0x3f;
  seq = seq & 0x1f;

  if (axis == 3) {
    auto& reqs = m_odrv_requests;
    if (reqs.seq_numbers[req_id] != seq) {
      return false;
    }
    uint8_t latency{0};
    if (payload_size > 0) {
      // response to read request
      reqs.setEndpointState((ODrvRequestId)req_id, EndpointState::Idle);
      latency = (uint8_t)timestamp - reqs.req_timestamps[req_id];
      processReadResponse((ODrvRequestId)req_id, payload);
      if (req_id == 2) {
        m_last_uptime_ts = timestamp;
      }
    }
    m_statistics.max_latency = std::max<uint16_t>(m_statistics.max_latency, latency);
  }

  if (axis == 1 || axis == 2) {
    axis = axis - 1;
    auto& reqs = m_axis_requests[axis];
    // check that sequence number correspond to request
    if (reqs.seq_numbers[req_id] != seq) {
      return false;
    }
    uint8_t latency{0};
    if (payload_size > 0) {
      // response to read request
      reqs.setEndpointState((AxisRequestId)req_id, EndpointState::Idle);
      latency = (uint8_t)timestamp - m_axis_requests[axis].req_timestamps[req_id];
      processReadResponse(axis, (AxisRequestId)req_id, payload);
    } else {
      // response to write request
      reqs.setEndpointState((AxisRequestId)req_id, EndpointState::Idle);
      latency = (uint8_t)timestamp - m_axis_requests[axis].req_timestamps[req_id];
    }
    m_statistics.max_latency = std::max<uint16_t>(m_statistics.max_latency, latency);
  }
  return true;
}

void ODriveClient::queueReadRequest(ODrvRequestId req_id, uint32_t timestamp) {
  uint8_t req_id_bin = static_cast<uint8_t>(req_id) | ((3) << 6);
  auto endpoint_id = c_odrv_endpoints[static_cast<uint8_t>(req_id)];

  uint16_t seq{0};

  auto& reqs = m_odrv_requests;

  switch (req_id) {
    case ODrvRequestId::VBus:
      seq = queueReadEndpoint<float>(endpoint_id, req_id_bin);
      break;
    case ODrvRequestId::IBus:
      seq = queueReadEndpoint<float>(endpoint_id, req_id_bin);
      break;
    case ODrvRequestId::Uptime:
      seq = queueReadEndpoint<uint32_t>(endpoint_id, req_id_bin);
      break;
    default:
      return;
  }
  reqs.setEndpointState(req_id, EndpointState::ReadPending);
  reqs.seq_numbers[static_cast<uint8_t>(req_id)] = (seq & 0x3f);
  reqs.req_timestamps[static_cast<uint8_t>(req_id)] = static_cast<uint8_t>(timestamp & 0xff);
}

void ODriveClient::queueReadRequest(int axis, AxisRequestId req_id, uint32_t timestamp) {
  uint8_t req_id_bin = static_cast<uint8_t>(req_id) | ((axis + 1) << 6);
  auto endpoint_id = c_endpoints[static_cast<uint8_t>(req_id)] + c_axis_base[axis];

  uint16_t seq{0};

  auto& reqs = m_axis_requests[axis];

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
  reqs.setEndpointState(req_id, EndpointState::ReadPending);
  reqs.seq_numbers[static_cast<uint8_t>(req_id)] = (seq & 0x3f);
  reqs.req_timestamps[static_cast<uint8_t>(req_id)] = static_cast<uint8_t>(timestamp & 0xff);
}

void ODriveClient::writeRequest(int axis, AxisRequestId req_id, uint32_t timestamp) {
  uint8_t req_id_bin = static_cast<uint8_t>(req_id) | ((axis + 1) << 6);
  auto endpoint_id = c_endpoints[static_cast<uint8_t>(req_id)] + c_axis_base[axis];
  auto& reqs = m_axis_requests[axis];

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
      m_telemetry_ack[axis * 4 + 3] = true;
      m_telemetry_timestamps[axis * 4 + 3] = (uint8_t)(m_current_timestamp % 256);
      break;
    case AxisRequestId::InputTorque:
      seq = writeEndpoint(endpoint_id, m_axis_requests[axis].input_torque, req_id_bin, true);
      break;
    case AxisRequestId::TorqueLimit:
      seq = writeEndpoint(endpoint_id, m_axis_requests[axis].torque_lim, req_id_bin, true);
      break;
    case AxisRequestId::PosEstimate:
      break;
    case AxisRequestId::VelEstimate:
      break;
    case AxisRequestId::CurrentIqSetpoint:
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
  reqs.setEndpointState(req_id, EndpointState::WritePending);

  reqs.seq_numbers[static_cast<uint8_t>(req_id)] = seq;
  reqs.req_timestamps[static_cast<uint8_t>(req_id)] = static_cast<uint8_t>(timestamp & 0xff);
}

void ODriveClient::processReadResponse(ODrvRequestId req_id, uint8_t* payload) {
  switch (req_id) {
    case ODrvRequestId::VBus:
      std::memcpy(&m_odrv_requests.vbus, payload, sizeof(float));
      break;
    case ODrvRequestId::Uptime: {
      uint32_t uptime = *(uint32_t*)payload;
      if (uptime < m_odrv_requests.uptime) {
        requestSynchronization();
      }
      m_odrv_requests.uptime = uptime;
    } break;
    default:
      break;
  }
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
      m_telemetry_ack[axis * 4 + 0] = true;
      m_telemetry_timestamps[axis * 4 + 0] = (uint8_t)(m_current_timestamp % 256);
      break;
    case AxisRequestId::VelEstimate:
      m_telemetry.axis[axis].vel_estimate = *(float*)payload;
      m_telemetry_ack[axis * 4 + 1] = true;
      m_telemetry_timestamps[axis * 4 + 1] = (uint8_t)(m_current_timestamp % 256);
      break;
    case AxisRequestId::CurrentIqSetpoint:
      m_telemetry.axis[axis].current_iq_setpoint = *(float*)payload;
      m_telemetry_ack[axis * 4 + 2] = true;
      m_telemetry_timestamps[axis * 4 + 2] = (uint8_t)(m_current_timestamp % 256);
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
