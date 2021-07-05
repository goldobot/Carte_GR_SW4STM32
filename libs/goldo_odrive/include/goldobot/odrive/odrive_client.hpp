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

  struct Statistics {
    uint16_t max_latency{0};
    uint16_t timeout_errors{0};
    uint32_t uptime;
    bool is_synchronized;
  };

  struct Config {
    uint16_t req_global_data_period{200};
    uint16_t req_errors_period{100};
    uint16_t req_axis_states_period{100};
    uint16_t req_telemetry_period{10};
    uint16_t req_set_vel_setpoints_period{10};
    uint16_t timeout{50};
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

 public:
  ODriveClient();
  // Set the exchange on which to queue ODrive packets
  void setOutputExchange(MessageExchange* exchange, CommMessageType message_type);

  // Process one ODrive response packet
  bool processResponse(uint32_t timestamp, sequence_number_t sequence_number, uint8_t* payload,
                       size_t payload_size);
  void doStep(uint32_t timestamp);

  void setMotorsEnable(bool motors_enable);
  void setVelocitySetPoint(int axis, float input_vel, float input_torque);
  void setTorqueLimit(int axis, float torque_lim);
  void setPIDConfig(const PIDConfig& config);
  bool startMotorsCalibration();  // run motors startup procedure
  void clearErrors();

  const std::array<AxisErrorState, 2>& errors() const noexcept;
  const Telemetry& telemetry() const noexcept;
  const std::array<AxisState, 2>& axisStates() const noexcept;
  const std::array<AxisCalibrationState, 2>& axisCalibrationStates() const noexcept;
  Statistics statistics();

  Statistics m_statistics;

  uint16_t m_user_endpoints[4] = {0, 0, 0, 0};
  float m_user_endpoints_values[4] = {0, 0, 0, 0};

  // private:
  enum class EndpointState {
    Idle,
    ReadRequested,   // should read endpoint
    ReadPending,     // read command sent, waiting for response
    WriteRequested,  // should write endpoint
    WritePending     // write command, waiting for response
  };
  enum class AxisRequestId {
    CurrentState = 0,  // state control
    RequestedState,
    ControlMode,
    InputVel,  // inputs
    InputTorque,
    TorqueLimit,
    PosEstimate,  // telemetry
    VelEstimate,
    CurrentIqSetpoint,
    AxisError,  // errors
    MotorError,
    ControllerError,
    EncoderError,
    SensorlessEstimatorError,
    VelGain,  // controller configuration
    VelIntegratorGain,
    EncoderIsReady,  // readiness status
    MotorIsArmed,
    MotorIsCalibrated,
    FetThermistorTemperature,  // temperature readings
    MotorThermistorTemperature,
    ClearErrors
  };

  enum class ODrvRequestId : uint8_t {
    VBus = 0,
    IBus,
    Uptime,
    RebootODrive,
    UserEndpoint0,
    UserEndpoint1,
    UserEndpoint2,
    UserEndpoint3
  };

  struct AxisPendingRequests {
    float input_vel{0};
    float input_torque{0};
    float torque_lim{0.4f};
    uint32_t requested_state{0};
    uint32_t control_mode{2};

    // value of 0 means no request pending
    uint8_t seq_numbers[21];
    uint8_t req_timestamps[21];
    uint32_t endpoint_states[3];

    inline EndpointState endpointState(AxisRequestId req_id) {
      int word_idx = static_cast<unsigned>(req_id) / 8;
      int shift = (static_cast<unsigned>(req_id) % 8) * 4;
      return static_cast<EndpointState>((endpoint_states[word_idx] >> shift) & 0xf);
    }

    inline void setEndpointState(AxisRequestId req_id, EndpointState state) {
      int word_idx = static_cast<unsigned>(req_id) / 8;
      int shift = (static_cast<unsigned>(req_id) % 8) * 4;

      endpoint_states[word_idx] =
          (endpoint_states[word_idx] & ~(0xf << shift)) | (static_cast<unsigned>(state) << shift);
    }
    inline void requestRead(AxisRequestId req_id) {
      if (endpointState(req_id) == EndpointState::Idle) {
        setEndpointState(req_id, EndpointState::ReadRequested);
      }
    };
    inline void requestWrite(AxisRequestId req_id) {
      if (endpointState(req_id) == EndpointState::Idle) {
        setEndpointState(req_id, EndpointState::WriteRequested);
      }
    };
  };

  struct ODrvPendingRequests {
    float vbus{0};
    float ibus{0};
    uint32_t uptime{0};
    uint8_t seq_numbers[3];
    uint8_t req_timestamps[3];
    uint32_t endpoint_states[1];

    inline EndpointState endpointState(ODrvRequestId req_id) {
      int word_idx = static_cast<unsigned>(req_id) / 8;
      int shift = (static_cast<unsigned>(req_id) % 8) * 4;
      return static_cast<EndpointState>((endpoint_states[word_idx] >> shift) & 0xf);
    }

    inline void setEndpointState(ODrvRequestId req_id, EndpointState state) {
      int word_idx = static_cast<unsigned>(req_id) / 8;
      int shift = (static_cast<unsigned>(req_id) % 8) * 4;

      endpoint_states[word_idx] =
          (endpoint_states[word_idx] & ~(0xf << shift)) | (static_cast<unsigned>(state) << shift);
    }
    inline void requestRead(ODrvRequestId req_id) {
      if (endpointState(req_id) == EndpointState::Idle) {
        setEndpointState(req_id, EndpointState::ReadRequested);
      }
    };
    inline void requestWrite(ODrvRequestId req_id) {
      if (endpointState(req_id) == EndpointState::Idle) {
        setEndpointState(req_id, EndpointState::WriteRequested);
      }
    };
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
  void queueReadRequest(ODrvRequestId req_id, uint32_t timestamp);
  void processReadResponse(int axis, AxisRequestId req_id, uint8_t* payload);
  void processReadResponse(ODrvRequestId req_id, uint8_t* payload);
  void writeRequest(int axis, AxisRequestId req_id, uint32_t timestamp);

  template <typename T>
  sequence_number_t writeEndpoint(endpoint_id_t endpoint, const T& val, uint8_t req_id,
                                  bool ack_requested = false);

  void processAxisResponse(int axis, uint16_t endpoint, uint8_t* payload, size_t payload_size);

  void requestSynchronization();
  void checkSynchronization();
  void updateAxisEndpoint(int axis, int req_idx, uint32_t timestamp, int& reqs_left);
  void updateEndpoints(uint32_t timestamp);

  MessageExchange* m_exchange{nullptr};
  CommMessageType m_request_packet_message_type;

  std::array<AxisState, 2> m_axis_states;
  Telemetry m_telemetry;
  std::array<AxisErrorState, 2> m_errors;
  std::array<AxisCalibrationState, 2> m_axis_calibration_states;
  AxisPendingRequests m_axis_requests[2];
  ODrvPendingRequests m_odrv_requests;

  Config m_config;
  // when desynchronized (after a timeout or on emergency stop button pushed), read all values from
  // the odrive
  bool m_is_synchronized{false};
  bool m_is_calibrated{false};
  uint32_t m_last_uptime_ts{0};

  uint8_t m_req_idx{0};
  uint32_t m_next_write_inputs_timestamp{0};
  uint32_t m_next_read_telemetry_timestamp{0};
  uint32_t m_next_read_global_state_timestamp{0};
  uint32_t m_next_read_states_timestamp{0};
  uint32_t m_next_read_errors_timestamp{0};

  uint16_t m_seq{1};  //! sequence id of next odrive request packet

  uint16_t m_odrive_calibration_state{0};  // 0 idle 1:calib axis0 2 calib axis1

  // base dnpoints id for axis0 and axis1
  static constexpr uint16_t c_axis_base[2] = {71, 300};

  // endpoint ids are relative to axis for axis specific endpoints
  static const uint16_t c_endpoints[22];
  // global endpoints
  static const uint16_t c_odrv_endpoints[4];

  // torque_constant in NM/A to convert between torque and motor current

  static constexpr uint16_t c_odrive_key{0x9b40};  // firmware 5.1

  static constexpr uint32_t c_odrive_consts_axis_state[3] = {
      1, 8, 3};  // idle, closed_loop, request calibration
  static constexpr uint32_t c_odrive_consts_control_mode{2};  // velocity control
};

}  // namespace goldobot
