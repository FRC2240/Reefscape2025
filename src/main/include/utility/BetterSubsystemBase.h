#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/phoenix6/TalonFX.hpp"

#include "utility/MotorUtils.h"

#include <vector>
#include <string>

// Maximum tries to apply a PID config to a motor. 
// If unsucessful, a message will be logged to the console
constexpr int MAX_CONFIG_APPLY_ATTEMPTS = 5;

class BetterSubsystemBase : public frc2::SubsystemBase
{
public:
  // Call in init function with motors and associated PIDs and names
  void AddPID(std::vector<MotorUtils::Motor> motors);
  void AddPID(MotorUtils::Motor motor);

  // Call inside of robotContainer setPID
  void SetPID();

  // Should run periodically
  void LogDashboard();

private:
  std::vector<MotorUtils::Motor> motors;
};