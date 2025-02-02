#pragma once
#include "subsystems/Elevator.h"

Elevator::Elevator()
{
    MotorUtils::Motor::LogValues logValues{true, true, true};
    MotorUtils::Motor ElevatorMotor{&m_motor, CONSTANTS::ELEVATOR::PidValue, logValues};
    AddPID(ElevatorMotor);
    SetPID();

    ctre::phoenix6::configs::TalonFXConfiguration elevator_config{};

    m_follower_motor.SetControl(ctre::phoenix6::controls::Follower(m_motor.GetDeviceID(), true));
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
        set_position(position); },
                            {this})
        .Until([this, pos]
               { return CONSTANTS::IN_THRESHOLD<units::angle::turn_t>(get_position(), pos, CONSTANTS::ELEVATOR::POSITION_THRESHOLD); });
}

frc2::CommandPtr Elevator::idle_command()
{
    return set_position_command(CONSTANTS::ELEVATOR::BOTTOM_POS);
};

frc2::CommandPtr Elevator::follow_joystick_command(frc2::CommandXboxController *stick)
{
    return frc2::RunCommand([this, stick]
                            {
        double stickpos = stick->GetLeftY(); // CHANGEME
        if (
            (stickpos < -CONSTANTS::ELEVATOR::DEADBAND_THRESHOLD && get_position() > CONSTANTS::ELEVATOR::BOTTOM_POS)
            || (stickpos > CONSTANTS::ELEVATOR::DEADBAND_THRESHOLD && get_position() < CONSTANTS::ELEVATOR::TOP_POS)
        ) {
            m_motor.SetControl(control_req.WithVelocity(CONSTANTS::ELEVATOR::JOYSTICK_SPEED * stickpos));
        } else {
            m_motor.SetControl(control_req.WithVelocity(0_tps));
        }; },
                            {this})
        .ToPtr();
};

units::angle::turn_t Elevator::get_position()
{
    return m_motor.GetPosition().GetValue();
};

void Elevator::set_position(units::angle::turn_t pos)
{
    m_motor.SetControl(control_req.WithPosition(pos));
};