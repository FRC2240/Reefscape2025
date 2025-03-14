#pragma once
#include "subsystems/Climber.h"

Climber::Climber()
{
    ctre::phoenix6::configs::TalonFXConfiguration conf{};
    conf.MotionMagic.MotionMagicAcceleration = 250_tr_per_s_sq;
    conf.MotionMagic.MotionMagicCruiseVelocity = 35_tps;
    conf.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
    m_motor.GetConfigurator().Apply(conf);
    SetPID();
    m_motor.SetPosition(CONSTANTS::CLIMBER::DEFAULT_POS);
}

void Climber::SetPID()
{
    MotorUtils::SetPID(m_motor, coeff);
}

void Climber::InitSendable(wpi::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("Climber");
    MotorUtils::BuildSender(builder, &coeff);
    MotorUtils::BuildSender(builder, &m_motor);
}

frc2::CommandPtr Climber::set_position_command(units::angle::turn_t pos)
{
    return frc2::RunCommand([this, pos]
                            { set_position(pos); },
                            {this})
        .Until([this, pos]

               { return CONSTANTS::IN_THRESHOLD<units::angle::turn_t>(get_angle(), pos, CONSTANTS::CLIMBER::POSITION_THRESHOLD); });
}

frc2::CommandPtr Climber::idle_command()
{
    return set_position_command(CONSTANTS::CLIMBER::DEFAULT_POS);
};

frc2::CommandPtr Climber::extend_command()
{
    return set_position_command(CONSTANTS::CLIMBER::EXTEND_POS);
};

frc2::CommandPtr Climber::climb_command()
{
    return set_position_command(CONSTANTS::CLIMBER::CLIMB_POS);
};

void Climber::set_position(units::angle::turn_t pos)
{
    m_motor.SetControl(control_req.WithPosition(pos));
};

units::degree_t Climber::get_angle()
{
    return m_motor.GetPosition().GetValue();
};
