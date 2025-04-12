#pragma once
#include "subsystems/FunnelPowered.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/Commands.h"

PoweredFun::PoweredFun() {

};

frc2::CommandPtr PoweredFun::spin(units::ampere_t current)
{
    return frc2::cmd::Run([this, current]
                          {
    ctre::phoenix6::controls::TorqueCurrentFOC req{current};
    m_motor.SetControl(req); }, {this});
};

void PoweredFun::monitor_temp(){
    frc::SmartDashboard::PutNumber("Funnel Temp", m_motor.GetDeviceTemp().GetValue().value());
    if (m_motor.GetDeviceTemp().GetValue().value() > 80)
    {
        my_alert.Set(true);
    } else
    {
        my_alert.Set(false);
    }
}