#include "Action.hpp"
using namespace User;

// 适配后的构造函数
Action::Action(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
               int motor_id, bool reverse)
  : LibXR::Application("ActionModule", hw, app),
    motor_id_(motor_id),
    reverse_(reverse),
    pos_pid_(5.0f, 0.1f, 0.03f, 80.0f, 4.0f),
    target_angle_(0.0f),
    is_loop_running_(false) {

  // 适配点：从ApplicationManager中获取ModuleManager，再创建Motor模块
  auto& module_mgr = app.GetModuleManager();
  motor_ = module_mgr.CreateModule<GM6020Motor>(hw, module_mgr, motor_id, reverse);
  
  if (!motor_) {
    LOG_ERROR("Action模块初始化失败：无法创建GM6020Motor实例");
    return;
  }

  LOG_INFO("Action模块初始化完成（仅45°旋转，电机ID：{}）", motor_id_);
}


void Action::Rotate45Degree() {
  if (!motor_) {
    LOG_ERROR("电机未初始化，无法旋转45°");
    return;
  }

  const auto& feedback = motor_->GetFeedback();
  if (!feedback.is_valid) {
    LOG_WARN("电机无反馈，默认从0°开始旋转45°");
    target_angle_ = 45.0f;
  } else {
    target_angle_ = LibXR::CycleValue<float>(0.0f, 360.0f).Get(feedback.angle + 45.0f);
  }

  is_loop_running_ = true;
  LOG_INFO("启动GM6020旋转45°，目标角度：{:.1f}°", target_angle_);
}

void Action::StopLoop() {
  is_loop_running_ = false;
  if (motor_) motor_->SetTarget(0.0f);
  LOG_INFO("45°旋转完成，电机已停止");
}

void Action::OnMonitor() override {
  if (!is_loop_running_ || !motor_) return;

  const auto& feedback = motor_->GetFeedback();
  if (!feedback.is_valid) return;

  float angle_error = LibXR::CycleValue<float>(0.0f, 360.0f).GetDiff(feedback.angle, target_angle_);
  if (fabs(angle_error) < 0.5f) {
    StopLoop();
    return;
  }

  float current_cmd = pos_pid_.Update(target_angle_, feedback.angle, 0.01f);
  current_cmd = reverse_ ? -current_cmd : current_cmd;
  motor_->SetTarget(current_cmd);
