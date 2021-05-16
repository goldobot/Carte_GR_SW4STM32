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
    uint16_t req_vel_estimates_period;
    uint16_t req_axis_errors_period;
    uint16_t req_motor_errors_period;
    uint16_t req_axis_states_period;
    uint16_t req_set_vel_setpoints_period;
    uint16_t timeout;
  };

 public:
  // Set the exchange on which to queue ODrive packets
  void setOutputExchange(MessageExchange* exchange, CommMessageType message_type);

  // Process one ODrive responbse packet
  bool processResponse(sequence_number_t sequence_number, uint8_t* payload, size_t payload_size);
  void doStep();

  void setMotorsEnable(bool motors_enable);
  void setVelocitySetPoint(int axis, float vel_setpoint, float current_feedforward, bool immediate);
  bool startMotorsCalibration();
  void clearErrors();

  float axisVelEstimate(int axis) const noexcept;
  uint32_t axisCurrentState(int axis) const noexcept;
  uint32_t axisError(int axis) const noexcept;
  uint32_t motorError(int axis) const noexcept;

 private:
  template <typename T>
  sequence_number_t queueReadEndpoint(endpoint_id_t endpoint);
  template <typename T>
  sequence_number_t writeEndpoint(endpoint_id_t endpoint, const T& val);

  MessageExchange* m_exchange{nullptr};
  CommMessageType m_request_packet_message_type;

  uint16_t m_seq{1};
  uint16_t m_odrive_calibration_state{0};  // 0 idle 1:calib axis0 2 calib axis1
  uint16_t m_seq_vel_estimate[2] = {0, 0};
  uint16_t m_seq_axis_current_state[2] = {0, 0};
  uint16_t m_seq_axis_error[2] = {0, 0};
  uint16_t m_seq_motor_error[2] = {0, 0};

  float m_axis_vel_estimate[2] = {0, 0};
  uint32_t m_axis_current_state[2] = {0, 0};
  uint32_t m_axis_error[2] = {0, 0};
  uint32_t m_motor_error[2] = {0, 0};

  static constexpr uint16_t c_axis0_base{71};
  static constexpr uint16_t c_axis1_base{300};

  //relative to axis
  static constexpr uint16_t c_endpoint_vel_gain{140}; // odrive velocity controller P coefficient
  static constexpr uint16_t c_endpoint_vel_integrator_gain{141}; // odrive velocity controller I coefficient
  static constexpr uint16_t c_endpoint_current_setpoint{78}; // axis.motor.current_control.Iq_setpoint, proportional to motor torque


 /* // Axis commands
  input_vel{124}
  input_torque{125};
  torque_lim{111};

  // Axis status
  axis_state{2};
  exis_error{0};
  motor_error{61};
  controller_error{122}
  encoder_error{165};

  //control
  requested_state{3};
  clear_errors{299};
*/




  // torque_constant in NM/A to convert between torque and motor current


  static constexpr uint16_t c_odrive_key{0x9b40};  // firmware 5.1
  static constexpr uint16_t c_endpoint_vel_estimate[2] = {249, 478};
  static constexpr uint16_t c_endpoint_axis_error[2] = {71, 300};
  static constexpr uint16_t c_endpoint_motor_error[2] = {132, 361};
  static constexpr uint16_t c_endpoint_input_vel[2] = {195, 424};
  static constexpr uint16_t c_endpoint_current_state[2] = {73, 302};
  static constexpr uint16_t c_endpoint_requested_state[2] = {74, 303};
  static constexpr uint16_t c_endpoint_control_mode[2] = {208, 437};
  static constexpr uint32_t c_odrive_consts_axis_state[3] = {
      1, 8, 3};  // idle, closed_loop, request calibration
  static constexpr uint32_t c_odrive_consts_control_mode{2};  // velocity control

  uint16_t m_cnt{0};
  uint16_t m_cnt_next_request_vel_estimates{0};
  uint16_t m_cnt_next_request_axis_errors{5};
  uint16_t m_cnt_next_request_motor_errors{10};
  uint16_t m_cnt_next_request_axis_states{15};
  uint16_t m_cnt_next_set_velocity_setpoints{20};
};

}  // namespace goldobot
