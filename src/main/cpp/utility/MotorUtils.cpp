#include "utility/MotorUtils.h"

MotorUtils::Motor::Motor(ctre::phoenix6::hardware::TalonFX *motor, CONSTANTS::PidCoeff coeff, MotorUtils::Motor::LogValues values)
    : motorPtr{motor}, pid{coeff}, logValues{values}
{
  ctre::phoenix6::configs::TalonFXConfiguration base_config{};
  base_config.Audio.BeepOnBoot = true;
  base_config.Audio.BeepOnConfig = true;

  base_config.CurrentLimits.SupplyCurrentLimitEnable = true;
  base_config.CurrentLimits.StatorCurrentLimitEnable = true;
  base_config.CurrentLimits.SupplyCurrentLimit = pid.currentLimits.supply;
  base_config.CurrentLimits.StatorCurrentLimit = pid.currentLimits.stator;
  
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
  frc::SmartDashboard::PutNumber("motors/" + name + "/LIMITS/supply", pid.currentLimits.supply.value());
  frc::SmartDashboard::PutNumber("motors/" + name + "/LIMITS/stator", pid.currentLimits.stator.value());
}

CONSTANTS::PidCoeff MotorUtils::Motor::GetDashboard()
{
  CONSTANTS::PidCoeff config;
  config.kS = frc::SmartDashboard::GetNumber("motors/" + name + "/PID/kS", pid.kS);
  config.kP = frc::SmartDashboard::GetNumber("motors/" + name + "/PID/kP", pid.kP);
  config.kI = frc::SmartDashboard::GetNumber("motors/" + name + "/PID/kI", pid.kI);
  config.kD = frc::SmartDashboard::GetNumber("motors/" + name + "/PID/kD", pid.kD);
  config.kG = frc::SmartDashboard::GetNumber("motors/" + name + "/PID/kG", pid.kG); 

  config.currentLimits.supply = units::ampere_t{frc::SmartDashboard::GetNumber("motors/" + name + "LIMITS/supply", pid.currentLimits.supply.value())};
  config.currentLimits.stator = units::ampere_t{frc::SmartDashboard::GetNumber("motors/" + name + "LIMITS/stator", pid.currentLimits.stator.value())};

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
    frc::SmartDashboard::PutNumber("motors/" + name + "/LOG/supply_current", motorPtr->GetSupplyCurrent().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("motors/" + name + "/LOG/stator_current", motorPtr->GetSupplyCurrent().GetValueAsDouble());
  }
}