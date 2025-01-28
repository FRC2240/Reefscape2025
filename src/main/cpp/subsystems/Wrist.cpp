#include "subsystems/Wrist.h"

#include "frc2/command/RunCommand.h"

Wrist::Wrist()
{
    MotorUtils::SetPID(m_motor, coeff);
    m_motor.SetPosition(CONSTANTS::WRIST::DEFAULT_POSITION);
}

void Wrist::SetPID() {
     MotorUtils::SetPID(m_motor, coeff);
}

void Wrist::InitSendable(wpi::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("Wrist");
    MotorUtils::BuildSender(builder, &coeff);
    MotorUtils::BuildSender(builder, &m_motor);
}

void Wrist::set_angle(units::angle::degree_t angle)
{   
    m_motor.SetControl(ctre::phoenix6::controls::PositionVoltage{angle});
}

units::degree_t Wrist::get_angle()
{
    return m_motor.GetPosition().GetValue();
}

frc2::CommandPtr Wrist::set_angle_command(units::degree_t pos)
{
    return frc2::RunCommand([this, pos]
                            { set_angle(pos); },
                            {this})
        .Until([this, pos] -> bool
               { CONSTANTS::IN_THRESHOLD<units::angle::degree_t>(get_angle(), pos, CONSTANTS::WRIST::POSITION_THRESHOLD); })
        .WithName("Set Wrist Angle");
}
