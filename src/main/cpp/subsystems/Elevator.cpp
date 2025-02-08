#pragma once
#include "subsystems/Elevator.h"
#include "frc/smartdashboard/SmartDashboard.h"

Elevator::Elevator()
{
    ctre::phoenix6::configs::TalonFXConfiguration conf{};
    conf.MotionMagic.MotionMagicAcceleration = 250_tr_per_s_sq;
    conf.MotionMagic.MotionMagicCruiseVelocity = 35_tps;
    conf.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
    m_motor.GetConfigurator().Apply(conf);

    SetPID();

    m_follower_motor.SetControl(ctre::phoenix6::controls::Follower(m_motor.GetDeviceID(), true));
}

void Elevator::SetPID()
{
    MotorUtils::SetPID(m_motor, coeff);
}

void Elevator::InitSendable(wpi::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("Elevator");
    MotorUtils::BuildSender(builder, &coeff);
    MotorUtils::BuildSender(builder, &m_motor);
}

frc2::CommandPtr Elevator::set_position_command(units::angle::turn_t pos)
{
    return frc2::RunCommand([this, pos]
                            {
        units::angle::turn_t position = pos;
        if (position > CONSTANTS::ELEVATOR::TOP_POS) {
            position = CONSTANTS::ELEVATOR::TOP_POS;
        } else if (position < CONSTANTS::ELEVATOR::BOTTOM_POS) {
            position = CONSTANTS::ELEVATOR::BOTTOM_POS;
        }
        frc::SmartDashboard::PutNumber("elv/desired", position.value());
        frc::SmartDashboard::PutNumber("elv/delta", m_motor.GetPosition().GetValueAsDouble() - position.value());
        set_position(position); },
                            {this})
        .Until([this, pos]
               { return CONSTANTS::IN_THRESHOLD<units::angle::turn_t>(get_position(), pos, CONSTANTS::ELEVATOR::POSITION_THRESHOLD * 3); });
}

frc2::CommandPtr Elevator::idle_command()
{
    return set_position_command(CONSTANTS::ELEVATOR::BOTTOM_POS);
};

units::angle::turn_t Elevator::get_position()
{
    return m_motor.GetPosition().GetValue();
};

void Elevator::set_position(units::angle::turn_t pos)
{
    m_motor.SetControl(control_req.WithPosition(pos));
};