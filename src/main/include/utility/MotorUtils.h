#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include <wpi/sendable/SendableBuilder.h>

#include "Constants.h"

#include <string>

namespace MotorUtils {

  // Maximum tries to apply a PID config to a motor. 
  // If unsucessful, a message will be logged to the console
  constexpr int MAX_CONFIG_APPLY_ATTEMPTS = 5;

  void SetPID(ctre::phoenix6::hardware::TalonFX& motor, CONSTANTS::PidCoeff pid);
  void BuildSender(wpi::SendableBuilder &builder, CONSTANTS::PidCoeff* coeff);
  void BuildSender(wpi::SendableBuilder &builder, ctre::phoenix6::hardware::TalonFX* motor);
};