#pragma once

#include "frc/DriverStation.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <frc2/command/button/CommandXboxController.h>
#include <frc/GenericHID.h>
#include <iostream>

namespace DriverUtils {
  // Put this in periodic
  void DriverTeleopPeriodic();
}