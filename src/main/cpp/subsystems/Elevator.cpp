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

    // Initialize the desired position to the current to ensure no movement
    desiredPosition = m_motor.GetPosition().GetValue();
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

frc2::CommandPtr Elevator::elevator_offset_command(units::angle::turn_t amount) {
    return frc2::RunCommand([this, amount] {
        desiredPosition += amount;
    }).ToPtr();
}

frc2::CommandPtr Elevator::set_position_command(units::angle::turn_t pos)
{
    // Sets the desired position to the value requested, resetting any offsets
    desiredPosition = pos;

    return frc2::RunCommand([this, pos] {
        // Clamps desiredPosition to the min and max elevator positions
        if (desiredPosition > CONSTANTS::ELEVATOR::TOP_POS) {
            desiredPosition = CONSTANTS::ELEVATOR::TOP_POS;
        } else if (desiredPosition < CONSTANTS::ELEVATOR::BOTTOM_POS) {
            desiredPosition = CONSTANTS::ELEVATOR::BOTTOM_POS;
        }
        // Dashboard logging
        frc::SmartDashboard::PutNumber("elv/desired", desiredPosition.value());
        frc::SmartDashboard::PutNumber("elv/delta", m_motor.GetPosition().GetValueAsDouble() - desiredPosition.value());

        // Sets the position on the motor
        set_position(desiredPosition); 
        
        }, {this}).ToPtr();
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