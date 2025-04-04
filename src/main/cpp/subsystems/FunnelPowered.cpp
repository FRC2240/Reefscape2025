#pragma once
#include "subsystems/FunnelPowered.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/Commands.h"


frc2::CommandPtr PoweredFun::spin(units::ampere_t current)
{
    ctre::phoenix6::controls::TorqueCurrentFOC req{current};
    m_motor.SetControl(req);
};