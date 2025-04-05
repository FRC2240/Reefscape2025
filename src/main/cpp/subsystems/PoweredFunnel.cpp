#pragma once
#include "subsystems/PoweredFunnel.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/Commands.h"

PoweredFunnel::PoweredFunnel() {

};

frc2::CommandPtr PoweredFunnel::spin(units::ampere_t current)
{
    ctre::phoenix6::controls::TorqueCurrentFOC req{current};
    m_motor.SetControl(req);
};