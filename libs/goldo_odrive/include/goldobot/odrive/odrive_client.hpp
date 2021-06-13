#pragma once
#include <chrono>
#include <cstdint>
#include <array>

namespace goldobot {
class MessageExchange;
enum class CommMessageType : uint16_t;

class ODriveClient {
 public:
  using endpoint_id_t = uint16_t;
  using sequence_number_t = uint16_t;

  struct Config {
    uint16_t req_errors_period{100};
    uint16_t req_axis_states_period{100};
    uint16_t req_telemetry_period{10};
    uint16_t req_set_vel_setpoints_period{10};
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

  struct AxisTelemetry {
    float pos_estimate{0};
    float vel_estimate{0};
    float current_iq_setpoint{0};
  };

  struct Telemetry {
    uint32_t timestamp{0};
    AxisTelemetry axis[2];
  };

  struct AxisState {
	uint32_t current_state{0};
	uint32_t requested_state{0};
	uint32_t control_mode{0};
  };

  struct AxisCalibrationState {
	bool encoder_is_ready{0};
  };

  enum class AxisRequestId {
	  CurrentState=0, // state control
	  RequestedState,
	  ControlMode,
	  InputVel, //inputs
	  InputTorque,
	  TorqueLimit,
	  PosEstimate, // telemetry
	  VelEstimate,
	  CurrentIqSetpoint,
	  AxisError, // errors
	  MotorError,
	  ControllerError,
	  EncoderError,
	  SensorlessEstimatorError,
	  VelGain, // controller configuration
	  VelIntegratorGain,
	  EncoderIsReady, // readiness status
	  MotorIsArmed,
	  MotorIsCalibrated,
	  FetThermistorTemperature, // temperature readings
	  MotorThermistorTemperature,
	  BusVoltage, // global data
	  BusCurrent,
	  ClearErrors,
	  RebootODrive
  };

 public:
  // Set the exchange on which to queue ODrive packets
  void setOutputExchange(MessageExchange* exchange, CommMessageType message_type);

  // Process one ODrive response packet
  bool processResponse(uint32_t timestamp, sequence_number_t sequence_number, uint8_t* payload, size_t payload_size);
  void doStep(uint32_t timestamp);

  void setMotorsEnable(bool motors_enable);
  void setVelocitySetPoint(int axis, float input_vel, float input_torque);
  void setPIDConfig(const PIDConfig& config);
  bool startMotorsCalibration();  // run motors startup procedure
  void clearErrors();

  const std::array<AxisErrorState, 2>& errors() const noexcept;
  const Telemetry& telemetry() const noexcept;
  const std::array<AxisState,2>& axisStates() const noexcept;
  const std::array<AxisCalibrationState, 2>& axisCalibrationStates() const noexcept;

 private:
  struct AxisPendingRequests {
    float input_vel;
    float input_torque;
    float torque_lim;
    uint32_t requested_state{0};
    uint32_t control_mode;

    // value of 0 means no request pending
    uint8_t seq_numbers[21];
    uint8_t req_timestamps[21];
    uint32_t read_pending_flags{0};
    uint32_t read_requested_flags{0x1fffff};
    uint8_t write_seq_numbers[21];
    uint8_t write_req_timestamps[21];
    uint32_t write_pending_flags{0};
    uint32_t write_requested_flags{0};
  };


  // req_id two most important bits are 0 for global data, 01 for axis 0, 10 for axis1
  // other bits are the AxisRequestId value
  // sequence ids are 0bcc dddd ddee eeee
  // MSB is 0 as defined in the odrive protocol
  // b is 1 for internal requests, 0 for raspberry requests
  // cc is the axis id
  // dddddd is the request id
  // eeeeee is a cyclic sequence id for error detection
  template <typename T>
  sequence_number_t queueReadEndpoint(endpoint_id_t endpoint, uint8_t req_id);

  void queueReadRequest(int axis, AxisRequestId req_id, uint32_t timestamp);
  void processReadResponse(int axis, AxisRequestId req_id, uint8_t* payload);
  void writeRequest(int axis, AxisRequestId req_id, uint32_t timestamp);

  template <typename T>
  sequence_number_t writeEndpoint(endpoint_id_t endpoint, const T& val, uint8_t req_id, bool ack_requested = false);

  void processAxisResponse(int axis, uint16_t endpoint, uint8_t* payload, size_t payload_size);

  MessageExchange* m_exchange{nullptr};
  CommMessageType m_request_packet_message_type;

  std::array<AxisState,2> m_axis_states;
  Telemetry m_telemetry;
  std::array<AxisErrorState, 2> m_errors;
  std::array<AxisCalibrationState, 2> m_axis_calibration_states;
  AxisPendingRequests m_axis_requests[2];
  uint16_t m_requested_state_flags{0};

  // flags: msb=1, request pending, 1 bit for each received ack
  // timeout, counter value of command timeout
  uint8_t m_flags_clear_errors{0};
  uint16_t m_timeout_clear_errors{0};

  uint8_t m_flags_set_motors_enable{0};
  uint16_t m_timeout_set_motors_enable{0};

  Config m_config;
  // when desynchronized (after a timeout or on emergency stop button pushed), read all values from the odrive
  bool m_is_synchronized{false};
  bool m_is_calibrated{false};

  uint8_t m_synchronize_idx{0};
  uint32_t m_synchronize_timestamp{0};

  uint8_t m_req_idx{0};
  uint32_t m_next_write_inputs_timestamp{0};
  uint32_t m_next_states_timestamp{0};

  uint16_t m_seq{1};  //! sequence id of next odrive request packet

  uint16_t m_odrive_calibration_state{0};  // 0 idle 1:calib axis0 2 calib axis1

  // base dnpoints id for axis0 and axis1
  static constexpr uint16_t c_axis_base[2] = {71, 300};

  // endpoint ids are relative to axis for axis specific endpoints
  static const uint16_t c_endpoints[25];


  static constexpr uint16_t c_endpoint_clear_errors{228};

  // torque_constant in NM/A to convert between torque and motor current

  static constexpr uint16_t c_odrive_key{0x9b40};  // firmware 5.1

  static constexpr uint32_t c_odrive_consts_axis_state[3] = {
      1, 8, 3};  // idle, closed_loop, request calibration
  static constexpr uint32_t c_odrive_consts_control_mode{2};  // velocity control
};

}  // namespace goldobot
