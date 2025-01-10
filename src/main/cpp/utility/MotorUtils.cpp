#include "utility/MotorUtils.h"

MotorUtils::Motor::Motor()
{
  ctre::phoenix6::configs::TalonFXConfiguration base_config{};
  base_config.Audio.BeepOnBoot = true;
  base_config.Audio.BeepOnConfig = true;
  base_config.CurrentLimits.StatorCurrentLimit = CONSTANTS::DEFAULT_CURRENT_LIMIT;
  name = motorPtr->GetDescription();
  motorPtr->GetConfigurator().Apply(base_config);
}

void MotorUtils::Motor::PutDashboard()
{
  frc::SmartDashboard::PutNumber("motors/" + name + "/PID/kS", pid.kS);
  frc::SmartDashboard::PutNumber("motors/" + name + "/PID/kP", pid.kP);
  frc::SmartDashboard::PutNumber("motors/" + name + "/PID/kI", pid.kI);
  frc::SmartDashboard::PutNumber("motors/" + name + "/PID/kD", pid.kD);
  frc::SmartDashboard::PutNumber("motors/" + name + "/PID/kG", pid.kG);
}

MotorUtils::PidCoeff MotorUtils::Motor::GetDashboard()
{
  PidCoeff config;
  config.kS = frc::SmartDashboard::GetNumber("motors/" + name + "/PID/kS", pid.kS);
  config.kP = frc::SmartDashboard::GetNumber("motors/" + name + "/PID/kP", pid.kP);
  config.kI = frc::SmartDashboard::GetNumber("motors/" + name + "/PID/kI", pid.kI);
  config.kD = frc::SmartDashboard::GetNumber("motors/" + name + "/PID/kD", pid.kD);
  config.kG = frc::SmartDashboard::GetNumber("motors/" + name + "/PID/kG", pid.kG);

  return config;
}

void MotorUtils::Motor::SetLogValues(MotorUtils::Motor::LogValues logValues)
{
  this->logValues.position = logValues.position;
  this->logValues.position = logValues.velocity;
  this->logValues.acceleration = logValues.acceleration;
  this->logValues.temp = logValues.temp;
  this->logValues.current = logValues.current;
}

void MotorUtils::Motor::LogDashboard()
{
  if (logValues.position)
  {
    auto position = motorPtr->GetPosition().GetValueAsDouble();
    frc::SmartDashboard::PutNumber("motors/" + name + "/LOG/position", position);
  }
  if (logValues.velocity)
  {
    auto velocity = motorPtr->GetVelocity().GetValueAsDouble();
    frc::SmartDashboard::PutNumber("motors/" + name + "/LOG/velocity", velocity);
  }
  if (logValues.acceleration)
  {
    auto acceleration = motorPtr->GetAcceleration().GetValueAsDouble();
    frc::SmartDashboard::PutNumber("motors/" + name + "/LOG/acceleration", acceleration);
  }
  if (logValues.temp)
  {
    frc::SmartDashboard::PutNumber("motors/" + name + "/LOG/temp", motorPtr->GetDeviceTemp().GetValueAsDouble());
  }
  if (logValues.current)
  {
    frc::SmartDashboard::PutNumber("motors/" + name + "/LOG/stator_current", motorPtr->GetStatorCurrent().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("motors/" + name + "/LOG/stator_current", motorPtr->GetSupplyCurrent().GetValueAsDouble());
  }
}