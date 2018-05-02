#include "goldobot/tasks/uart_comm.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/propulsion/odometry_config.hpp"
#include "goldobot/robot.hpp"
#include <stdio.h>
#include <stdarg.h>
#include <cstring>
#include <cstdlib>
#include <cmath>

using namespace goldobot;

UARTCommTask::UARTCommTask()
{
}

const char* UARTCommTask::name() const
{
	return "uart_comm";
}

void UARTCommTask::taskFunction()
{
	setvbuf(stdin, NULL, _IONBF, 0);
	while(1)
	{
		printf("[Choose mode]\r\n");
		printf("1: Test encoders\r\n");
		printf("2: Test motors\r\n");
		printf("3: Test odometry\r\n");
		printf("4: Calibrate odometry\r\n");
		printf("5: Test propulsion\r\n");
		printf("5: Test path planner\r\n");
		printf("\n");
		char c = 0;

		prompt_char(">: ", &c);
		switch(c)
		{
		case '1':
			loop_test_encoders();
			break;
		case '2':
			loop_test_motors();
			break;
		case '3':
			loop_test_odometry();
			break;
		case '4':
			loop_calibrate_odometry();
			break;
		case '5':
			loop_test_propulsion();
			break;
		default:
			break;
		}
	}
}

void UARTCommTask::printf(const char* format, ...)
{
	// Format string
	va_list args;
	va_start(args, format);
	int num_chars = vsnprintf(m_buffer, c_buffer_size, format, args);
	va_end(args);
	if(num_chars > 0)
	{
		Hal::uart_transmit(0,m_buffer,num_chars);
	}
}

bool UARTCommTask::prompt_char(const char* prompt, char* c)
{
	printf(prompt);
	const char* line = read_line();
	bool status = false;
	if(strlen(line) == 1)
	{
		*c = line[0];
		status = true;
	}
	printf("\r\n");
	return status;
}

bool UARTCommTask::prompt_int(const char* prompt, int* c)
{
	printf(prompt);
	printf("\r\n");
	const char* line = read_line();
	*c = atoi(line);
	return true;
}

bool UARTCommTask::peek_char(char* c)
{
	return Hal::uart_read_char(0, c,false);
}

const char* UARTCommTask::read_line()
{
	m_buffer_index = 0;
	while(m_buffer_index < c_buffer_size)
	{
		char c = read_char();
		if(c == '\r')
		{
			break;
		}

		if(c == '\177' || c == '\b')
		{
			if(m_buffer_index > 0)
			{
				Hal::uart_transmit(0, &c, 1);
				m_buffer_index--;
			}

		}
		else
		{
			Hal::uart_transmit(0, &c, 1);
			m_buffer[m_buffer_index] = c;
			m_buffer_index++;
		}
	}
	m_buffer[m_buffer_index] = '\0';
	return m_buffer;
}

char UARTCommTask::read_char()
{
	char c;
	Hal::uart_read_char(0,&c,true);
	return c;
}

void UARTCommTask::loop_test_encoders()
{
	printf("[Test encoders]\r\n");

	while(1)
	{
		uint16_t left;
		uint16_t right;
		Hal::read_encoders(left, right);
		printf("%i,%i        \r",left,right);

		char c = 0;
		if(peek_char(&c))
		{
			printf("\r\n");
			return;
		}
		delayTicks(100);
	}
}


void UARTCommTask::loop_test_odometry()
{
	printf("[Test odometry]\r\n");
	while(1)
		{
			auto pose = Robot::instance().odometry().pose();
			printf("x= %.3fm, y= %.3fm, yaw=%.1fdeg, speed= %.3f m.s-1        \r",pose.position.x, pose.position.y, pose.yaw*180/M_PI, pose.speed);
			//printf("%.3f\n", pose.position.x);
			char c = 0;
			if(peek_char(&c))
			{
				printf("\n");
				return;
			}
			delayTicks(100);
		}
}


void UARTCommTask::loop_calibrate_odometry()
{
	OdometryConfig odometry_config = Robot::instance().odometryConfig();
	while(1)
	{
		printf("[Calibrate odometry]\r\n");
		printf("dist_per_count_left: %.6e\r\n", odometry_config.dist_per_count_left);
		printf("dist_per_count_right: %.6e\r\n", odometry_config.dist_per_count_right);
		printf("wheel_spacing: %.6e\r\n", odometry_config.wheel_spacing);
		printf("1: calibrate translation\r\n");
		printf("2: calibrate rotation\r\n");
		printf("q: quit\r\n");
		char choice = 0;
		prompt_char(">: ", &choice);
		switch(choice)
		{
		case '1':
		{
			printf("Advance the robot 50cm and press a key\n");
			int32_t left_total;
			int32_t right_total;
			loop_measure_encoders_delta(&left_total, &right_total);
			printf("counts_left: %i, counts_right: %i\n",left_total,right_total);
			odometry_config.dist_per_count_left = 0.5f / left_total;
			odometry_config.dist_per_count_right = 0.5f / right_total;

		}
			break;
		case '2':
		{
			printf("Rotate the robot one time to the left and press a key\n");
			int32_t left_total;
			int32_t right_total;
			loop_measure_encoders_delta(&left_total, &right_total);
			printf("counts_left: %i, counts_right: %i\n",left_total,right_total);
			float dist = fabs(right_total * odometry_config.dist_per_count_right - left_total * odometry_config.dist_per_count_left);
			odometry_config.wheel_spacing = dist / (M_PI * 2);
		}
		break;
		case 'q':
			return;
		}
	}
}

void UARTCommTask::loop_measure_encoders_delta(int32_t* left_o, int32_t* right_o)
{
	*left_o = 0;
	*right_o = 0;
	uint16_t old_left;
	uint16_t old_right;
	char c = 0;
	Hal::read_encoders(old_left, old_right);
	while(!peek_char(&c) )
	{
		uint16_t left;
		uint16_t right;

		Hal::read_encoders(left, right);

		int diff_left = left - old_left;
		int diff_right = right - old_right;

		old_left = left;
		old_right = right;

		if(diff_left > 4096)
		{
			diff_left -= 8192;
		}
		if(diff_left < -4096)
		{
			diff_left += 8192;
		}
		if(diff_right > 4096)
		{
			diff_right -= 8192;
		}
		if(diff_right < -4096)
		{
			diff_right += 8192;
		}
		*left_o += diff_left;
		*right_o += diff_right;
	}
}

void UARTCommTask::loop_test_motors()
{
	while(1)
	{
		printf("[Test motors]\r\n");
		printf("1: enable motors\r\n");
		printf("2: disable motors\r\n");
		printf("3: set pwm [-100-100]\r\n");
		printf("q: quit\r\n");
		char choice = 0;
		prompt_char(">d: ", &choice);
		switch(choice)
		{
		case '1':
			Hal::set_motors_enable(true);
			break;
		case '2':
			Hal::set_motors_enable(false);
			break;
		case '3':
			int left_pwm, right_pwm;
			prompt_int("Left motor pwm: ",&left_pwm);
			prompt_int("Right motor pwm: ",&right_pwm);
			Hal::set_motors_pwm(left_pwm * 0.01, right_pwm * 0.01);
			break;
		case '4':
		{
			int pwm = 0;
			float speed_trace[200];
			prompt_int("motor pwm: ",&pwm);
			Hal::set_motors_enable(true);
			Hal::set_motors_pwm(pwm * 0.01, pwm * 0.01);
			for(int i =0; i< 200; i++)
			{
				if(i < 100)
				{
					Hal::set_motors_pwm(pwm * 0.01 * i/100, pwm * 0.01 * i/100);

				} else
				{
					Hal::set_motors_pwm(pwm * 0.01 * (200-i)/100, pwm * 0.01 * (200-i)/100);
				}
				auto pose = Robot::instance().odometry().pose();
				speed_trace[i] = pose.speed;
				delayTicks(10);
			}
			Hal::set_motors_pwm(0,0);
			for(int i =0; i< 200; i++)
			{
				printf("%f, \n", speed_trace[i]);
			}

		}

			break;
		case 'q':
			return;
		}
	}
}

void UARTCommTask::loop_test_propulsion()
{
	goldobot::PropulsionController* controller = &(Robot::instance().propulsion());

	while(1)
		{
			printf("[Test propulsion]\r\n");
			printf("1: enable motors\r\n");
			printf("2: disable motors\r\n");
			printf("3: test_line\r\n");
			printf("4: test repositioning\r\n");
			printf("q: quit\r\n");
			char choice = 0;
			prompt_char(">: ", &choice);
			switch(choice)
			{
			case '1':
				Hal::set_motors_enable(true);
				break;
			case '2':
				Hal::set_motors_enable(false);
				break;
			case '3':
			{
				Vector2D points[3];
				auto pose = Robot::instance().propulsion().target_pose();
				points[0] = pose.position;
				points[1] = points[0];
				points[1].x += 0.5 * cos(pose.yaw);
				points[1].y += 0.5 * sin(pose.yaw);
				points[2] = points[1];
				points[2].x += 0.5 * sin(pose.yaw);
				points[2].y += 0.5 * cos(pose.yaw);
				Robot::instance().propulsion().executeTrajectory(points, 2, PropulsionController::Direction::Forward, 0.3,0.5,0.5);
				while(Robot::instance().propulsion().state() == PropulsionController::State::FollowTrajectory)
				{
					delayTicks(100);
				}
				printf("target_x,");
				printf("target_y,");
				printf("target_yaw,");
				printf("target_speed,");
				printf("robot_x,");
				printf("robot_y,");
				printf("robot_yaw,");
				printf("robot_speed,");
				printf("robot_yaw_rate,");
				printf("pwm_left,");
				printf("pwm_right\r\n");
				delayTicks(10);
				for(unsigned i = 0; i < controller->m_dbg_index;i++)
				{
					printf("%f,",controller->m_dbg_target_position_buffer[i].x);
					printf("%f,",controller->m_dbg_target_position_buffer[i].y);
					printf("%f,",controller->m_dbg_target_yaw_buffer[i]);
					printf("%f,",controller->m_dbg_target_speed_buffer[i]);
					printf("%f,",controller->m_dbg_pose_buffer[i].position.x);
					printf("%f,",controller->m_dbg_pose_buffer[i].position.y);
					printf("%f,",controller->m_dbg_pose_buffer[i].yaw);
					printf("%f,",controller->m_dbg_pose_buffer[i].speed);
					printf("%f,",controller->m_dbg_pose_buffer[i].yaw_rate);
					printf("%f,",controller->m_dbg_left_pwm_buffer[i]);
					printf("%f,",controller->m_dbg_right_pwm_buffer[i]);
					printf("\r\n");
					delayTicks(10);
				}
				printf("\n");
			}
				break;
			case '4':
				Robot::instance().propulsion().executeRepositioning(PropulsionController::Direction::Forward, 0.1,{0,0},0);
				break;
			case 'q':
				return;
			}
		}
}

