#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "Constants.h"

#include <vector>
#include <string>

namespace MotorUtils {
  struct Motor {

    struct LogValues {
      bool position = false;
      bool velocity = false;
      bool acceleration = false;
      bool temp = true;
      bool current = true;
    };


    Motor(ctre::phoenix6::hardware::TalonFX* motor, CONSTANTS::PidCoeff coeff, LogValues values);
    Motor(ctre::phoenix6::hardware::TalonFX* motor, CONSTANTS::PidCoeff coeff, CONSTANTS::PidCoeff::CurrentLimits currentLimits, LogValues values);

    ctre::phoenix6::hardware::TalonFX *motorPtr;
    CONSTANTS::PidCoeff pid;
    CONSTANTS::PidCoeff::CurrentLimits currentLimits;
    LogValues logValues;

    std::string name;
    
    void PutDashboard();
    CONSTANTS::PidCoeff GetDashboard();

    void SetLogValues(LogValues logValues);
    void LogDashboard(); // Sets log values in the dashboard. Should be run periodically
  };
};