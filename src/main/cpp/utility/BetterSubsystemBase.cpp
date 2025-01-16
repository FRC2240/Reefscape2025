#include "utility/BetterSubsystemBase.h"
#include "frc/DataLogManager.h"
#include <string>
#include <iostream>


void BetterSubsystemBase::AddPID(MotorUtils::Motor motor) {
  std::vector<MotorUtils::Motor> motorVec {motor};
  AddPID(motorVec);
}

void BetterSubsystemBase::AddPID(std::vector<MotorUtils::Motor> _motors) {
  for (MotorUtils::Motor &motor : _motors) {
    this->motors.push_back(motor);
  }

  for (MotorUtils::Motor &motor : this->motors) {
    motor.PutDashboard();
  }
}

void BetterSubsystemBase::SetPID() {
  for (MotorUtils::Motor &motor: motors) {
    ctre::phoenix6::configs::TalonFXConfiguration configs{};
    CONSTANTS::PidCoeff PIDValue = motor.GetDashboard();
    configs.Slot0.kS = PIDValue.kS; 
    configs.Slot0.kP = PIDValue.kP; 
    configs.Slot0.kI = PIDValue.kI; 
    configs.Slot0.kD = PIDValue.kD; 
    configs.Slot0.kG = PIDValue.kG;
    configs.CurrentLimits.SupplyCurrentLimit = PIDValue.currentLimits.supply;
    configs.CurrentLimits.StatorCurrentLimit = PIDValue.currentLimits.stator;

    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int j = 0; j < MAX_CONFIG_APPLY_ATTEMPTS; j++)
    {
        status = motor.motorPtr->GetConfigurator().Apply(configs);
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
  for (MotorUtils::Motor &motor : motors) {
    motor.LogDashboard();
  }
}
