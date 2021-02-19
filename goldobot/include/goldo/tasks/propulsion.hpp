#pragma once
#include "goldo/tasks/task.hpp"
#include "goldo/propulsion/simple_odometry.hpp"
#include "goldo/propulsion/controller.hpp"
#include "goldo/core/message_queue.hpp"

#include <cstdint>

namespace goldobot
{
  class PropulsionTask : public Task
  {
  public:
    PropulsionTask();
    const char* name() const override;

    SimpleOdometry& odometry();
    PropulsionController& controller();

  private:
    MessageQueue m_message_queue;
    unsigned char m_message_queue_buffer[512];

    MessageQueue m_urgent_message_queue;
    unsigned char m_urgent_message_queue_buffer[768];

    SimpleOdometry m_odometry;
    PropulsionController m_controller;
    uint16_t m_encoder_left{0};
    uint16_t m_encoders_right{0};
    uint16_t m_telemetry_counter{0};
    PropulsionController::State m_previous_state{PropulsionController::State::Inactive};

    bool m_adversary_detection_enabled{true};

    void doStep();
    void processMessage();
    void processUrgentMessage();
    void taskFunction() override;

    void onMsgExecuteTrajectory();
    void onMsgExecutePointTo();

    void measureNormal(float angle, float distance);
    void measurePointLongi(Vector2D point, float sensor_offset);
  };
}
