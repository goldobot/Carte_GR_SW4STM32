#include "hal/generic/hal.hpp"
#include "goldo/tasks/arms.hpp"
#include "goldo/robot.hpp"

#include <string.h>
#include <algorithm>
#include "FreeRTOS.h"
#include "task.h"

using namespace goldobot;

/** Defines         **/
/** Instruction Set **/
#define AX_PING                     1
#define AX_READ_DATA                2
#define AX_WRITE_DATA               3
#define AX_REG_WRITE                4
#define AX_ACTION                   5
#define AX_RESET                    6
#define AX_SYNC_WRITE               131


struct DynamixelPacketHeader
{
  uint16_t magic;//0xFFFF
  uint8_t id;
  uint8_t length;
  uint8_t command;
};

void ArmsTask::dynamixels_transmit_packet(uint8_t id,  uint8_t command,  unsigned char* parameters, uint8_t num_parameters)
{
  DynamixelPacketHeader& header = *reinterpret_cast<DynamixelPacketHeader*>(m_dynamixels_buffer);
  header.magic = 0xFFFF;
  header.id = id;
  header.length = num_parameters + 2;
  header.command = command;
  memcpy(m_dynamixels_buffer+5, parameters, num_parameters);

  uint8_t checksum = 0;
  for(unsigned i = 2; i < 5 + num_parameters;i++)
  {
    checksum += m_dynamixels_buffer[i];
  }
  checksum = ~checksum;
  m_dynamixels_buffer[5 + num_parameters] = checksum;
  Hal::set_gpio(3, 1);
  Hal::uart_transmit(1,(char*)m_dynamixels_buffer,6 + num_parameters, true);
  Hal::set_gpio(3, 0);
}

DynamixelStatusError ArmsTask::dynamixels_receive_packet()
{
  memset(m_dynamixels_buffer, 0, 255);
  Hal::uart_receive(1,(char*)m_dynamixels_buffer,256, false);
  uint16_t bytes_received = 0;
  for(unsigned i=0;i<10;i++)
  {
    bytes_received = Hal::uart_bytes_received(1);

    // search for magic 0xFF
    if(bytes_received >= 4 && bytes_received >= m_dynamixels_buffer[3] + 4)
    {
      Hal::uart_receive_abort(1);

      // Check checksum
      m_dynamixels_receive_id = m_dynamixels_buffer[2];
      m_dynamixels_receive_num_parameters = m_dynamixels_buffer[3] - 2;
      m_dynamixels_receive_error = m_dynamixels_buffer[4];
      m_dynamixels_receive_params = m_dynamixels_buffer + 5;

      uint8_t checksum = 0;
      for(unsigned i = 2; i < 5 + m_dynamixels_receive_num_parameters;i++)
      {
        checksum += m_dynamixels_buffer[i];
      }
      checksum = ~checksum;

      if(checksum == m_dynamixels_buffer[5 + m_dynamixels_receive_num_parameters])
      {
        return DynamixelStatusError::Ok;
      } else
      {
        return DynamixelStatusError::ChecksumError;
      }
    }
    delay(1);
  }
  Hal::uart_receive_abort(1);
  return DynamixelStatusError::TimeoutError;
}

ArmsTask::ArmsTask():
  m_message_queue(m_message_queue_buffer, sizeof(m_message_queue_buffer))
{
}

const char* ArmsTask::name() const
{
  return "arms";
}

void ArmsTask::taskFunction()
{
  Robot::instance().mainExchangeIn().subscribe({72,80,&m_message_queue});
  Robot::instance().mainExchangeIn().subscribe({160,170,&m_message_queue});

  // \todo hack,for now we are using a harcoded configuration
  m_arm_state = ArmState::Idle;

  unsigned char buff[2];
  buff[0] = 0;
  buff[1] = (unsigned char)m_arm_state;
  Robot::instance().mainExchangeOut().pushMessage(CommMessageType::ArmsStateChange, buff, 2);

  int servo_idx = 0;
  while(1)
  {
    uint32_t clock = xTaskGetTickCount();
    auto prev_state = m_arm_state;
    if(m_arm_state == ArmState::Moving && clock >= m_end_move_timestamp )
    {
      m_arm_state = ArmState::Idle;
    }
    while(m_message_queue.message_ready() && m_arm_state != ArmState::Moving)
    {
      process_message();
      // Periodically check servo positions and torques
    }

    dynamixels_read_data(m_config.servos[servo_idx].id,0x24,(unsigned char*)&m_current_state[servo_idx], 6);
    unsigned char buff[8];
    buff[0] = m_config.servos[servo_idx].id;
    buff[1] = 0x24;
    memcpy(buff+2, &m_current_state[servo_idx], 6);
    Robot::instance().mainExchangeOut().pushMessage(CommMessageType::DbgDynamixelGetRegisters, buff, 8);

    servo_idx++;
    if(servo_idx == m_config.num_servos)
    {
      servo_idx = 0;
    }

    if(m_arm_state != prev_state)
    {
      unsigned char buff[2];
      buff[0] = 0;// arm id, 0 for now since we have only one arm
      buff[1] = (unsigned char)m_arm_state;
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::ArmsStateChange, buff, 2);
      Robot::instance().mainExchangeIn().pushMessage(CommMessageType::ArmsStateChange, buff, 2);
    }
    delay_periodic(1);
  }
}

void ArmsTask::shutdown()
{
  unsigned char buff = 0;
  dynamixels_write_data(254, 0x18, &buff, 1);
}

bool ArmsTask::dynamixels_read_data(uint8_t id, uint8_t address, unsigned char* buffer, uint8_t size)
{
  unsigned char tmp_buff[2];
  tmp_buff[0] = address;
  tmp_buff[1] = size;
  dynamixels_transmit_packet(id, AX_READ_DATA, tmp_buff, 2);
  auto received = dynamixels_receive_packet();
  if(received == DynamixelStatusError::Ok)
  {
    memcpy(buffer, m_dynamixels_buffer + 5, size);
  }
  return received == DynamixelStatusError::Ok;
}

bool ArmsTask::dynamixels_write_data(uint8_t id, uint8_t address, unsigned char* buffer, uint8_t size)
{
  unsigned char tmp_buff[16];
  tmp_buff[0] = address;
  memcpy(tmp_buff+1, buffer, size);
  dynamixels_transmit_packet(id, AX_WRITE_DATA, tmp_buff, size + 1);
  bool received = dynamixels_receive_packet() == DynamixelStatusError::Ok && m_dynamixels_receive_error==0;
  return received;
}

bool ArmsTask::dynamixels_reg_write(uint8_t id, uint8_t address, unsigned char* buffer, uint8_t size)
{
  unsigned char tmp_buff[32];
  tmp_buff[0] = address;
  memcpy(tmp_buff+1, buffer, size);
  dynamixels_transmit_packet(id, AX_REG_WRITE, tmp_buff, size + 1);
  bool received = dynamixels_receive_packet() == DynamixelStatusError::Ok && m_dynamixels_receive_error == 0;
  return received;
}

void ArmsTask::dynamixels_action()
{
  dynamixels_transmit_packet(0xFE, AX_ACTION, nullptr, 0);
}

void ArmsTask::go_to_position(uint8_t pos_id, uint16_t speed_percent, int torque_settings)
{
  int pos_idx = pos_id * m_config.num_servos;
  if(speed_percent<10)
  {
    speed_percent = 10;
  }

  // Launch dynamixels
  //uint8_t servo_ids[] = {81,82,1};
  //uint8_t servo_types[3] = {1,1,0};

  //get previous positions and compute move timing based on limiting speed
  uint16_t prev_posa[8];

  uint32_t tim = 1;

  for(int i=0; i< m_config.num_servos;i++)
  {
    uint16_t prev_pos = m_current_position[i];
    uint16_t tar_pos = m_config.positions_ptr[pos_idx+i];
    const auto& servo = m_config.servos[i];

    dynamixels_read_data(servo.id,0x24,(unsigned char*)&prev_pos, 2);
    prev_posa[i] = prev_pos;
    int diff_angle = abs(tar_pos - prev_pos);
    switch(servo.type)
    {
    case ServoType::DynamixelAX12:
      //ax 12
      tim = std::max<uint16_t>((diff_angle*25000)/(57 * 0x3ff), tim);
      break;
    case ServoType::DynamixelMX28:
      //mx28
      tim = std::max<uint16_t>((diff_angle*128)/0x3ff, tim);
      break;
    default:
      break;
    }
  }

  uint32_t time_ms = (tim * 100) / speed_percent;
  if(time_ms > 5000)
  {
    time_ms = 5000;
  }
  for(int i=0; i< m_config.num_servos;i++)
  {
    uint16_t buff[3];
    uint16_t prev_pos = m_current_position[i];
    buff[0] = m_config.positions_ptr[pos_idx+i]; // position setpoint
    buff[2] = m_config.torques_ptr[torque_settings * m_config.num_servos + i];

    prev_pos = prev_posa[i];

    int diff_angle = abs(buff[0] - prev_pos);

    switch(m_config.servos[i].type)
    {
    case ServoType::DynamixelAX12:
      //ax 12
      buff[1] = std::min<uint16_t>((diff_angle*25000)/(time_ms*57), 0x3ff);
      break;
    case ServoType::DynamixelMX28:
      //mx28
      buff[1] = std::min<uint16_t>((diff_angle*128)/(time_ms), 0x3ff);
      break;
    default:
      break;
    }
    if(buff[1] < 32)
    {
      buff[1] = 32;
    }
    // Write new register values
    int k = 0;
    while(!dynamixels_reg_write(m_config.servos[i].id,0x1E,(unsigned char*)buff, 6) && k < 5)
    {
      k++;
      delay(2);
    }
  }
  dynamixels_action();
  // Do it twice in case of error
  dynamixels_action();

  // Set time of end
  m_arm_state = ArmState::Moving;
  m_end_move_timestamp = xTaskGetTickCount() + (uint32_t)time_ms;
}

void ArmsTask::process_message()
{
  auto message_size = m_message_queue.message_size();
  switch(m_message_queue.message_type())
  {

  case CommMessageType::DbgDynamixelsList:
    {
      m_message_queue.pop_message(nullptr, 0);
      uint8_t buff[4] = {25,1};
      for(unsigned id = 0; id < 0xFE; id++)
      {
        if(dynamixels_read_data(id,0 , buff, 4))
        {
          Robot::instance().mainExchangeOut().pushMessage(CommMessageType::DbgDynamixelDescr, (unsigned char*)buff, 4);
        }
      }
    }
    break;
  case CommMessageType::DbgDynamixelSetTorqueEnable:
    {
      unsigned char buff[2];
      m_message_queue.pop_message(buff, 2);
      // Torque enable
      dynamixels_write_data(buff[0], 0x18, buff+1, 1);
    }
    break;
  case CommMessageType::DbgDynamixelSetGoalPosition:
    {
      unsigned char buff[3];
      m_message_queue.pop_message(buff, 3);
      // Goal position
      dynamixels_write_data(buff[0], 0x1E, buff+1, 2);
    }
    break;
  case CommMessageType::DbgDynamixelSetTorqueLimit:
    {
      unsigned char buff[3];
      m_message_queue.pop_message(buff, 3);
      // Goal position
      dynamixels_write_data(buff[0], 0x22, buff+1, 2);
    }
    break;
  case CommMessageType::DbgDynamixelGetRegisters:
    {
      unsigned char buff[3];
      unsigned char data_read[64];

      m_message_queue.pop_message(buff, 3);
      memcpy(data_read, buff, 2);
      if(dynamixels_read_data(buff[0], buff[1], data_read+2, buff[2]))
      {
        Robot::instance().mainExchangeOut().pushMessage(CommMessageType::DbgDynamixelGetRegisters, (unsigned char*)data_read, buff[2]+2);
      }
    }
    break;
  case CommMessageType::DbgDynamixelSetRegisters:
    {
      unsigned char buff[128];
      uint16_t size = m_message_queue.message_size();
      m_message_queue.pop_message(buff, 128);
      //id, addr, data
      if(dynamixels_write_data(buff[0], buff[1], buff+2, size-2));
    }
    break;
  case CommMessageType::DbgArmsSetPose:
    {
      unsigned char buff[16];
      m_message_queue.pop_message((unsigned char*)buff,(m_config.num_servos + 1) * sizeof(uint16_t));
      uint16_t* ptr = m_config.positions_ptr + m_config.num_servos*buff[1];
      memcpy(ptr, (unsigned char*)(buff+2), m_config.num_servos * sizeof(uint16_t));
    }
    break;
  case CommMessageType::DbgArmsSetTorques:
    {
      uint16_t buff[4];
      m_message_queue.pop_message((unsigned char*)buff,8);
      //uint16_t* ptr = m_config.m_torque_settings + 3*buff[0];
      //memcpy(ptr, (unsigned char*)(buff+1), 6);
    }
    break;
  case CommMessageType::DbgArmsGoToPosition:
    {
      unsigned char buff[4];
      m_message_queue.pop_message((unsigned char*)buff,4);
      go_to_position(buff[0], *(uint16_t*)(buff+2), buff[1]);
    }
    break;
  case CommMessageType::ArmsShutdown:
    m_message_queue.pop_message(nullptr, 0);
    shutdown();
    break;
  case CommMessageType::DynamixelSendPacket:
    {
      unsigned char buff[64];
      unsigned char data_read[64];

      m_message_queue.pop_message(buff, 64);
      memcpy(data_read, buff, 2);

      dynamixels_transmit_packet(buff[0], buff[1], buff+2, message_size-2);
      auto status = dynamixels_receive_packet();

      buff[0] = (unsigned char) status;
      memcpy(buff+1, m_dynamixels_buffer, 20);

      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::DynamixelStatusPacket, (unsigned char*)buff, 21);
    }
    break;
  default:
    m_message_queue.pop_message(nullptr, 0);
    break;
  }

}

