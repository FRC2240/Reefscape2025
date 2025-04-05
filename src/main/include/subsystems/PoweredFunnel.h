#pragma once

#include "Constants.h"
#include "utility/MotorUtils.h"

#include <units/angle.h>
#include <units/angular_velocity.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "frc2/command/CommandPtr.h"
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/RunCommand.h>
#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>

class PoweredFunnel : public frc2::SubsystemBase
{
public:
    PoweredFunnel();
    frc2::CommandPtr spin(units::ampere_t current = 40_A);

    frc2::CommandPtr coast();

private:
    ctre::phoenix6::hardware::TalonFX m_motor{CONSTANTS::POWEREDFUNNEL::POWEREDFUNNEL_ID};
};