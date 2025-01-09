#include "utility/MotorUtils.h"

void MotorUtils::Motor::PutDashboard()
{
  std::string name = motorPtr->GetDescription();
  frc::SmartDashboard::PutNumber(name + "/PID/kS", pid.kS);
  frc::SmartDashboard::PutNumber(name + "/PID/kP", pid.kP);
  frc::SmartDashboard::PutNumber(name + "/PID/kI", pid.kI);
  frc::SmartDashboard::PutNumber(name + "/PID/kD", pid.kD);
  frc::SmartDashboard::PutNumber(name + "/PID/kG", pid.kG);
}

MotorUtils::PIDValues MotorUtils::Motor::GetDashboard()
{
  std::string name = motorPtr->GetDescription();
  PIDValues config;
  config.kS = frc::SmartDashboard::GetNumber(name + "/PID/kS", pid.kS);
  config.kP = frc::SmartDashboard::GetNumber(name + "/PID/kP", pid.kP);
  config.kI = frc::SmartDashboard::GetNumber(name + "/PID/kI", pid.kI);
  config.kD = frc::SmartDashboard::GetNumber(name + "/PID/kD", pid.kD);
  config.kG = frc::SmartDashboard::GetNumber(name + "/PID/kG", pid.kG);

  return config;
}

void MotorUtils::Motor::SetLogValues(MotorUtils::Motor::LogValues logValues) {
  this->logValues.position = logValues.position;
  this->logValues.position = logValues.velocity;
  this->logValues.acceleration = logValues.acceleration;
}

void MotorUtils::Motor::LogDashboard() {
  std::string name = motorPtr->GetDescription();
  if (logValues.position) {
    auto position = motorPtr->GetPosition().GetValueAsDouble();
    frc::SmartDashboard::PutNumber(name + "/LOG/position", position);
  }
  if (logValues.velocity) {
    auto velocity = motorPtr->GetVelocity().GetValueAsDouble();
    frc::SmartDashboard::PutNumber(name + "/LOG/velocity", velocity);
  }
  if (logValues.acceleration) {
    auto acceleration = motorPtr->GetAcceleration().GetValueAsDouble();
    frc::SmartDashboard::PutNumber(name + "/LOG/acceleration", acceleration);
  }
}