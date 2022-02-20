#pragma once
#include "goldobot/core/geometry.hpp"
#include "goldobot/core/pid_controller.hpp"
#include "goldobot/core/trajectory_buffer.hpp"
#include "goldobot/core/trapezoidal_speed_profile.hpp"
#include "goldobot/propulsion/controller_config.hpp"
#include "goldobot/propulsion/low_level_controller.hpp"
#include "goldobot/propulsion/speed_controller.hpp"
#include "goldobot/propulsion/blocking_detector.hpp"

#include <cstdint>
#include <functional>

namespace goldobot {
class SimpleOdometry;
class TrajectoryBuffer;

namespace messages {

struct PropulsionTelemetry {
  int16_t x;  // quarters of mm
  int16_t y;
  int16_t yaw;
  int16_t speed;     // mm per second
  int16_t yaw_rate;  // mradian per second
  int16_t acceleration;
  int16_t angular_acceleration;
  uint16_t left_encoder;
  uint16_t right_encoder;
  int8_t left_pwm;  // percents
  int8_t right_pwm;
  uint8_t state;
  uint8_t error;
};

struct PropulsionTelemetryEx {
  int16_t target_x;  // quarters of mm
  int16_t target_y;
  int16_t target_yaw;
  int16_t target_speed;     // mm per second
  int16_t target_yaw_rate;  // mradian per second
  int16_t lookahead_x;      // quarters of mm
  int16_t lookahead_y;
  int16_t longitudinal_error;
  int16_t lateral_error;
  int16_t speed_error;
  int16_t yaw_error;
  int16_t yaw_rate_error;
};

}  // namespace messages

struct PropulsionConfiguration {
  float lookahead_distance;
  float lookahead_time;
};

class PropulsionController {
 public:
  // Current propulsion controller state
  enum class State : uint8_t {
    Inactive,          // Controller desactive
    Stopped,           // Robot immobile
    FollowTrajectory,  // Poursuite de trajectoire
    Rotate,            // Rotation sur place
    Reposition,        // Se deplace a vitesse fixe jusqu'a bloquer
    ManualControl,     // Controle manuel, utilise pour le reglage de PID
    EmergencyStop,     // Arret d'urgence, deccelere avant de s'arreter
    Error              // Controlleur desactive suite a une erreur
  };

  enum class Error {
    None,
    EmergencyStop,  // Un arret d'urgence s'est produit
    RobotBlocked,   // Blocage durant la derniere commande
    TrackingError   // Erreur de poursuite trop grande.
  };

  enum class Direction { Forward, Backward };

  enum class EventType : uint8_t { Unknown, Reposition, User = 33 };

  // message sent for an event
  // such as a slipping or blocking detection, a successfull repositioning, or a sensor detection
  struct Event {
    RobotPose pose;
    float parameter;
    uint32_t data[2];
    EventType type;
  };

 public:
  PropulsionController(SimpleOdometry* odometry);

  //! \brief Set enabled state
  void setEnable(bool enabled);

  //! \brief Get current controller state
  State state() const;

  //! \brief Returns true if controller state since last call
  bool stateChanged();

  //! \brief Get current controller error code
  Error error() const;

  //! \brief Clear error and return to Stopped state
  void clearError();

  //! \brief Return true if a command was completed during last update
  bool commandFinished();

  // \brief Return target robot pose
  const RobotPose& targetPose() const;

  // \brief Return current robot pose
  const RobotPose& currentPose() const;

  const LowLevelController& lowLevelController() const { return m_low_level_controller; };

  void update();

  float leftMotorVelocityInput() const noexcept;
  float rightMotorVelocityInput() const noexcept;

  float leftMotorTorqueInput() const noexcept;
  float rightMotorTorqueInput() const noexcept;

  float leftMotorTorqueLimit() const noexcept;
  float rightMotorTorqueLimit() const noexcept;

  void setMotorsVelEstimates(float left, float right);
  void setMotorsTorqueEstimates(float left, float right);

  void setAccelerationLimits(float accel, float deccel, float angular_accel, float angular_deccel);
  void prepareReposition(float distance, float speed);
  void setTargetSpeed(float speed);

  //! \brief reset robot pose. Only works if state is Inactive or Stopped. Also change odometry.
  bool resetPose(float x, float y, float yaw);

  bool executeTrajectory(Vector2D* points, int num_points, float speed);
  bool executeRepositioning(float speed, float accel);
  bool executePointTo(Vector2D target, float yaw_rate);
  bool executePointToBack(Vector2D target, float yaw_rate);
  bool executeMoveTo(Vector2D target, float speed);
  bool executeRotation(float delta_yaw, float yaw_rate);

  bool executeFaceDirection(float direction, float yaw_rate);
  bool executeTranslation(float distance, float speed);

  //! \brief Emergency stop. Abort current PointTo of FollowTrajectory command and bring the robot
  //! to a stop.
  void emergencyStop();

  void enterManualControl();
  void exitManualControl();

  void setTargetPose(const RobotPose& target_pose);
  void setControlLevels(uint8_t longi, uint8_t yaw);

  const PropulsionControllerConfig& config() const;
  void setConfig(const PropulsionControllerConfig& config);

  void setEventCallback(std::function<void(const Event&)>&& callback);
  void sendEvent(EventType type, uint32_t data1 = 0, uint32_t data2 = 0);

  messages::PropulsionTelemetry getTelemetry() const;
  /*< */
  messages::PropulsionTelemetryEx getTelemetryEx() const;

  propulsion::BlockingDetector m_blocking_detector;

 private:
  SimpleOdometry* m_odometry;
  PropulsionControllerConfig m_config;
  std::function<void(const Event&)> m_event_callback;

  LowLevelController m_low_level_controller;
  RobotPose m_current_pose;
  RobotPose m_target_pose;

  State m_state{State::Inactive};
  Error m_error{Error::None};
  bool m_command_finished{false};
  bool m_state_changed{false};
  bool m_emergency_stop{false};

  float m_reposition_speed{0};
  float m_reposition_distance{0};
  float m_accel{1};
  float m_deccel{1};
  float m_angular_accel{1};
  float m_angular_deccel{1};

  // \todo should split management of parametrized trajectory in separate class
  TrajectoryBuffer m_trajectory_buffer;
  float m_begin_yaw;              // yaw at beginning of current PointTo command
  float m_rotation_direction{1};  // sign of delta yaw
  float m_direction_sign{0};      // direction of rotation or trajectory
  SpeedController m_speed_controller;

  Direction m_direction;

  uint32_t m_time_base_ms{0};

  Vector2D m_lookahead_position;

  bool m_reposition_hit;
  uint32_t m_reposition_end_ts{0};

  //! \brief compute motors pwm values when the robot is static. Use PID controllers on yaw and
  //! longitudinal position
  void updateMotorsPwm();

  //! \brief Update current and lookahead positions in TrajectoryFollowing mode
  void updateTargetPositions();

  //! \brief Update target yaw and yaw rate in PointTo mode
  void updateTargetYaw();
  void updateReposition();
  bool detectBlocking();
  void check_tracking_error();

  void setState(State state);
  void setState(State state, Error error);

  // void initMoveCommand(float speed);

  void on_stopped_enter();
  void on_command_finished();

  void onFollowTrajectoryEnter();
  void onRepositionEnter();
  void onRepositionExit();
};
}  // namespace goldobot
