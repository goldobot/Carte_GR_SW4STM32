#pragma once
#include <chrono>
#include <cstdint>

namespace goldobot {
class MessageExchange;
enum class CommMessageType : uint16_t;

class ODriveClient {
 public:
  using endpoint_id_t = uint16_t;
  using sequence_number_t = uint16_t;

  struct Config {
    uint16_t req_errors_period{20};
    uint16_t req_axis_states_period{20};
    uint16_t req_telemetry_period{20};
    uint16_t req_set_vel_setpoints_period{20};
    uint16_t timeout;
  };

  struct PIDConfig {
    float vel_gain{0};
    float vel_integrator_gain{0};
  };

  struct AxisErrorState {
    uint32_t axis{0};
    uint32_t motor{0};
    uint32_t controller{0};
    uint32_t encoder{0};
    uint32_t sensorless_estimator{0};
  };

  struct Errors {
    AxisErrorState axis[2];
  };

  struct AxisTelemetry {
    float pos_estimate{0};
    float vel_estimate{0};
    float current_iq_setpoint{0};
  };

  struct Telemetry {
    uint32_t timestamp{0};
    AxisTelemetry axis[2];
  };

  struct AxisStates {
    uint32_t axis[2] = {0, 0};
  };

  enum class State : uint8_t {

  };

 public:
  // Set the exchange on which to queue ODrive packets
  void setOutputExchange(MessageExchange* exchange, CommMessageType message_type);

  // Process one ODrive response packet
  bool processResponse(sequence_number_t sequence_number, uint8_t* payload, size_t payload_size);
  void doStep(uint32_t timestamp);

  void setMotorsEnable(bool motors_enable);
  void setVelocitySetPoint(int axis, float input_vel, float input_torque);
  bool startMotorsCalibration();  // run motors startup procedure
  void clearErrors();

  const Errors& errors() const noexcept;
  const Telemetry& telemetry() const noexcept;
  const AxisStates axisStates() const noexcept;

 private:
  struct AxisPendingRequests {
    float input_vel;
    float input_torque;
    float torque_lim;
    uint32_t requested_state{0};
    uint32_t control_mode;

    uint16_t seq_current_state{0};
    uint16_t seq_requested_state{0};
    uint16_t seq_axis_error{0};
    uint16_t seq_motor_error{0};
    uint16_t seq_controller_error{0};
    uint16_t seq_encoder_error{0};
    uint16_t seq_sensorless_estimator_error{0};
    uint16_t seq_pos_estimate{0};
    uint16_t seq_vel_estimate{0};
    uint16_t seq_current_iq_setpoint{0};
    uint16_t seq_clear_errors{0};
  };

  template <typename T>
  sequence_number_t queueReadEndpoint(endpoint_id_t endpoint);
  template <typename T>
  sequence_number_t writeEndpoint(endpoint_id_t endpoint, const T& val, bool ack_requested = false);

  void processAxisResponse(int axis, uint16_t endpoint, uint8_t* payload, size_t payload_size);

  MessageExchange* m_exchange{nullptr};
  CommMessageType m_request_packet_message_type;

  AxisStates m_axis_states;
  Telemetry m_telemetry;
  Errors m_errors;
  AxisPendingRequests m_reqs[2];
  uint16_t m_requested_state_flags{0};

  // flags: msb=1, request pending, 1 bit for each received ack
  // timeout, counter value of command timeout
  uint8_t m_flags_clear_errors{0};
  uint16_t m_timeout_clear_errors{0};

  uint8_t m_flags_set_motors_enable{0};
  uint16_t m_timeout_set_motors_enable{0};

  Config m_config;

  uint16_t m_seq{1};  //! sequence id of next odrive request packet

  uint16_t m_odrive_calibration_state{0};  // 0 idle 1:calib axis0 2 calib axis1

  static constexpr uint16_t c_axis_base[2] = {71, 300};

  // relative to axis
  static constexpr uint16_t c_endpoint_current_state{2};
  static constexpr uint16_t c_endpoint_requested_state{3};
  static constexpr uint16_t c_endpoint_control_mode{137};

  static constexpr uint16_t c_endpoint_axis_error{0};
  static constexpr uint16_t c_endpoint_motor_error{61};
  static constexpr uint16_t c_endpoint_controller_error{122};
  static constexpr uint16_t c_endpoint_encoder_error{165};
  static constexpr uint16_t c_endpoint_sensorless_estimator{203};

  static constexpr uint16_t c_endpoint_pos_estimate{172};
  static constexpr uint16_t c_endpoint_vel_estimate{178};
  static constexpr uint16_t c_endpoint_current_iq_setpoint{78};

  static constexpr uint16_t c_endpoint_input_vel{124};
  static constexpr uint16_t c_endpoint_input_torque{125};
  static constexpr uint16_t c_endpoint_torque_lim{111};

  static constexpr uint16_t c_endpoint_vel_gain{140};  // odrive velocity controller P coefficient
  static constexpr uint16_t c_endpoint_vel_integrator_gain{
      141};  // odrive velocity controller I coefficient
  static constexpr uint16_t c_endpoint_clear_errors{228};

  // torque_constant in NM/A to convert between torque and motor current

  static constexpr uint16_t c_odrive_key{0x9b40};  // firmware 5.1

  static constexpr uint32_t c_odrive_consts_axis_state[3] = {
      1, 8, 3};  // idle, closed_loop, request calibration
  static constexpr uint32_t c_odrive_consts_control_mode{2};  // velocity control

  uint16_t m_cnt{0};
  uint16_t m_cnt_next_request_telemetry{0};
  uint16_t m_cnt_next_request_errors{0};
  uint16_t m_cnt_next_request_axis_states{0};
  uint16_t m_cnt_next_set_velocity_setpoints{0};
};

}  // namespace goldobot
