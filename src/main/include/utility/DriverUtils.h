#pragma once

#include "frc/DriverStation.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <frc2/command/button/CommandXboxController.h>
#include <frc/GenericHID.h>
#include <iostream>

// Duration of time that the controller will rumble at 45s left
constexpr double ALERT_RUMBLE_DURATION = 1.0;

namespace DriverUtils {
  // Put this in periodic
  void DriverTeleopPeriodic();
}