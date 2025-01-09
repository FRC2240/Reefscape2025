#include "utility/BetterSubsystemBase.h"
#include "frc/DataLogManager.h"
#include <string>
#include <iostream>

constexpr int MAX_CONFIG_APPLY_ATTEMPTS = 5;

void BetterSubsystemBase::AddPID(MotorUtils::Motor motor) {
  std::vector<MotorUtils::Motor> motorVec {motor};
  AddPID(motorVec);
}

void BetterSubsystemBase::AddPID(std::vector<MotorUtils::Motor> motors) {
  this->motors = motors;

  for (MotorUtils::Motor &const motor : this->motors) {
    motor.PutDashboard();
  }
}

void BetterSubsystemBase::SetPID() {
  for (int i = 0; i < motors.size(); i++) {
    ctre::phoenix6::configs::TalonFXConfiguration configs{};
    MotorUtils::PIDValues PIDValue = motors.at(i).GetDashboard();
    configs.Slot0.kS = PIDValue.kS; 
    configs.Slot0.kP = PIDValue.kP; 
    configs.Slot0.kI = PIDValue.kI; 
    configs.Slot0.kD = PIDValue.kD; 

    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int j = 0; j < MAX_CONFIG_APPLY_ATTEMPTS; j++)
    {
        status = motors.at(i).motorPtr->GetConfigurator().Apply(configs);
        if (status.IsOK())
            break;
    }
    if (!status.IsOK())
    {
      std::string errString = status.GetName();
      frc::DataLogManager::Log("Could not apply configs, error code: " + errString);
    }
  }
}

void BetterSubsystemBase::LogDashboard() {
  for (MotorUtils::Motor &const motor : motors) {
    motor.LogDashboard();
  }
}
