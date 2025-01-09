#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/phoenix6/TalonFX.hpp"

#include <vector>
#include <string>

namespace MotorUtils {
  struct PIDValues {
    double kS = 0;
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double kG = 0;
  };
  
  // If a motor has a name of "", it will use the device name from tunerX
  struct Motor {
    struct LogValues {
      bool position = false;
      bool velocity = false;
      bool acceleration = false;
    };

    ctre::phoenix6::hardware::TalonFX *motorPtr;
    PIDValues pid;
    LogValues logValues;
    
    void PutDashboard();
    PIDValues GetDashboard();

    void SetLogValues(LogValues logValues);
    void LogDashboard(); // Sets log values in the dashboard. Should be run periodically
  };
};