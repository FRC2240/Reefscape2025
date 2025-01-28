#pragma once
#include "subsystems/Elevator.h"
#include "cmath"
#include "utility/ForceLog.h"

Elevator::Elevator()
{
    MotorUtils::Motor::LogValues logValues{true, true, true, true, true};
    MotorUtils::Motor ElevatorMotor{&m_motor, CONSTANTS::ELEVATOR::PidValue, logValues};
    AddPID(ElevatorMotor);
    SetPID();

    m_motor_follow.SetControl(ctre::phoenix6::controls::Follower{m_motor.GetDeviceID(), 1});

    // ctre::phoenix6::configs::TalonFXConfiguration Elevator_config{};
    // m_motor.GetConfigurator().Apply(Elevator_config)
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
        set_position(position); },
                            {this})
        .Until([this, pos] -> bool
               { return CONSTANTS::IN_THRESHOLD(get_position(), pos, CONSTANTS::ELEVATOR::POSTITION_THRESHOLD); })
        .WithName("Elevator Set Pos");
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
            ctre::phoenix6::controls::VelocityTorqueCurrentFOC req{CONSTANTS::ELEVATOR::JOYSTICK_SPEED * stickpos};
            m_motor.SetControl(req);
        } else {
            m_motor.SetControl(ctre::phoenix6::controls::VelocityTorqueCurrentFOC{0_tps});
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
    // Soft limits on top & bottom
    try
    {
        std::clamp(pos, CONSTANTS::ELEVATOR::LIMITS::TOP, CONSTANTS::ELEVATOR::LIMITS::BOTTOM);
        ForceLog::debug(m_motor.SetControl(m_position_control.WithPosition(pos)).GetName());
    }
    catch (const std::exception &e)
    {
        ForceLog::fatal(e.what());
    }
};

frc2::CommandPtr Elevator::stress_test_up()
{
    bool going_up = 1;

    return frc2::cmd::Run([this, &going_up]
                          {
        if (going_up)
        {
            set_position(CONSTANTS::ELEVATOR::LIMITS::TOP);
            ForceLog::debug(std::to_string(counter));
        }
        else
        {
            set_position(CONSTANTS::ELEVATOR::LIMITS::BOTTOM);
        }
        if (get_position() >= CONSTANTS::ELEVATOR::LIMITS::TOP - 2_tr)
        {
            going_up = 0;
            counter++;
        }
        else if (get_position() <= CONSTANTS::ELEVATOR::LIMITS::BOTTOM + 2_tr)
        {
            going_up = 1;
        } }, {this});
}

frc2::CommandPtr Elevator::stress_test_down()
{
    return set_position_command(CONSTANTS::ELEVATOR::LIMITS::BOTTOM);
}