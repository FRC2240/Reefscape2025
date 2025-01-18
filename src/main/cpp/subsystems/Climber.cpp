#pragma once
#include "subsystems/Climber.h"


Climber::Climber() {
    MotorUtils::Motor::LogValues logValues {true, true, true};
    MotorUtils::Motor climberMotor{&m_motor, CONSTANTS::CLIMBER::PidValue, logValues};
    AddPID(climberMotor);
    SetPID();

    m_motor.SetPosition(CONSTANTS::CLIMBER::DEFAULT_POS);
}

frc2::CommandPtr Climber::set_position_command(units::angle::turn_t pos) {
    return frc2::RunCommand([this, pos] {
        set_position(pos);
    },
    {this}).ToPtr();
}

frc2::CommandPtr Climber::idle_command() {
    return set_position_command(CONSTANTS::CLIMBER::DEFAULT_POS);
};

frc2::CommandPtr Climber::extend_command(){
    return set_position_command(CONSTANTS::CLIMBER::EXTEND_POS);
};

frc2::CommandPtr Climber::climb_command(){
    return set_position_command(CONSTANTS::CLIMBER::CLIMB_POS);
};

void Climber::set_position(units::angle::turn_t pos) {
    m_motor.SetControl(ctre::phoenix6::controls::PositionTorqueCurrentFOC{pos});
};

units::degree_t Climber::get_angle()
{
  return m_motor.GetPosition().GetValue();
};

