#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "Constants.h"

#include <vector>
#include <string>

namespace MotorUtils {
  struct PidCoeff {
    double kP = 0; // Proportion
    double kI = 0; // Integral
    double kD = 0; // Derivative
    double kS = 0; // Static gain
    double kG = 0; // Gravity gain

    double min = -1; // Minimum output for control loop
    double max = 1;  // Maximum output for control loop
  };
  
  struct Motor {
    Motor();
    struct LogValues {
      bool position = false;
      bool velocity = false;
      bool acceleration = false;
      bool temp = true;
      bool current = true;
    };

    ctre::phoenix6::hardware::TalonFX *motorPtr;
    PidCoeff pid;
    LogValues logValues;
    std::string name;
    
    void PutDashboard();
    PidCoeff GetDashboard();

    void SetLogValues(LogValues logValues);
    void LogDashboard(); // Sets log values in the dashboard. Should be run periodically
  };
};