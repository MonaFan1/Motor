#pragma once
// clang-format on

#include <cstdint>
#include "app_framework.hpp"
#include "libxr_def.hpp"

/**
 * @brief 电机抽象接口
 */
class Motor {
 public:
  enum ControlMode : uint8_t {
    MODE_POSITION,
    MODE_VELOCITY,
    MODE_TORQUE,
    MODE_CURRENT,
    MODE_MIT,
  };

  struct MotorCmd {
    ControlMode mode;
    float reduction_ratio = 1.0f;
    float torque = 0;
    float position = 0;
    float velocity = 0;
    float kp = 0;
    float kd = 0;
  };

  struct Feedback {
    uint8_t error_id;
    uint8_t state = 0;
    float position;
    float velocity;
    float omega;
    float torque;
    float temp;
  };

  virtual ~Motor() = default;

  virtual void Enable() = 0;
  virtual void Disable() = 0;
  virtual void Relax() = 0;
  virtual ErrorCode Update() = 0;
  virtual const Feedback& GetFeedback() = 0;
  virtual void Control(const MotorCmd& cmd) = 0;
  virtual void ClearError() = 0;
  virtual void SaveZeroPoint() = 0;

 private:
};
