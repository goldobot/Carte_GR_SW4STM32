#include <cstdint>
#include <cstddef>

namespace goldobot {

class ODriveClient
{
public:
	bool processODriveResponse(uint8_t sequence_number, uint8_t* payload, size_t payload_size);


private:
	  uint16_t m_odrive_seq{1};
	  uint16_t m_odrive_calibration_state{0}; // 0 idle 1:calib axis0 2 calib axis1

	  uint16_t m_odrive_seq_axis0_vel_estimate{0};
	  uint16_t m_odrive_seq_axis1_vel_estimate{0};

	  uint16_t m_odrive_seq_axis0_current_state{0};
	  uint16_t m_odrive_seq_axis1_current_state{0};

	  uint16_t m_odrive_seq_axis0_error{0};
	  uint16_t m_odrive_seq_axis1_error{0};

	  uint16_t m_odrive_seq_axis0_motor_error{0};
	  uint16_t m_odrive_seq_axis1_motor_error{0};

	  float m_odrive_axis0_vel_estimate{0};
	  float m_odrive_axis1_vel_estimate{0};

	  uint32_t m_odrive_axis0_current_state{0};
	  uint32_t m_odrive_axis1_current_state{0};

	  uint32_t m_odrive_axis0_error{0};
	  uint32_t m_odrive_axis1_error{0};

	  uint32_t m_odrive_axis0_motor_error{0};
	  uint32_t m_odrive_axis1_motor_error{0};

	  static constexpr uint16_t c_odrive_key{0x9b40};  // firmware 5.1
	  static constexpr uint16_t c_odrive_endpoint_vel_estimate[2] = {249, 478};
	  static constexpr uint16_t c_odrive_endpoint_axis_error[2] = {71, 300};
	  static constexpr uint16_t c_odrive_endpoint_motor_error[2] = {132, 361};
	  static constexpr uint16_t c_odrive_endpoint_input_vel[2] = {195, 424};
	  static constexpr uint16_t c_odrive_endpoint_current_state[2] = {73, 302};
	  static constexpr uint16_t c_odrive_endpoint_requested_state[2] = {74, 303};
	  static constexpr uint16_t c_odrive_endpoint_control_mode[2] = {208, 437};
	  static constexpr uint32_t c_odrive_consts_axis_state[3] = {1, 8, 3};  // idle, closed_loop, request calibration
	  static constexpr uint32_t c_odrive_consts_control_mode = 2;        // velocity control
	  uint16_t m_odrive_cnt{0};                                          // send one message every 10 ms



};


}
