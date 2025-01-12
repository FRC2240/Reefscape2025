#pragma once
#include "subsystems/Climber.h"


Climber::Climber() {
    MotorUtils::Motor::LogValues logValues {true, true, true};
    MotorUtils::Motor climberMotor{&m_motor, CONSTANTS::CLIMBER::PidValue, logValues};
    AddPID(climberMotor);
    SetPID();

    m_motor.SetPosition(CONSTANTS::CLIMBER::NORM_POS);
}

frc2::CommandPtr Climber::setPositionCommand(units::angle::turn_t pos) {
    return frc2::RunCommand([this, pos] {
        setPosition(pos);
    },
    {this}).ToPtr();
}

frc2::CommandPtr Climber::idlecommand() {
    return setPositionCommand(CONSTANTS::CLIMBER::NORM_POS);
};

frc2::CommandPtr Climber::Extend(){
    return setPositionCommand(CONSTANTS::CLIMBER::EXTEND_POS);
};

frc2::CommandPtr Climber::climb(){
    return setPositionCommand(CONSTANTS::CLIMBER::CLIMB_POS);
};

void Climber::setPosition(units::angle::turn_t pos) {
    m_motor.SetControl(ctre::phoenix6::controls::PositionVoltage{pos});
};

units::degree_t Climber::get_angle()
{
  return m_motor.GetPosition().GetValue();
};

