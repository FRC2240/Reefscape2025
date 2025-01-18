#pragma once
#include "subsystems/Climber.h"


Climber::Climber() {
    MotorUtils::Motor::LogValues logValues {true, true, true};
    MotorUtils::Motor climberMotor{&m_motor, CONSTANTS::CLIMBER::PidValue, logValues};
    AddPID(climberMotor);
    SetPID();

    m_motor.SetPosition(CONSTANTS::CLIMBER::DEFAULT_POS);
}

frc2::CommandPtr Climber::SetPositionCommand(units::angle::turn_t pos) {
    return frc2::RunCommand([this, pos] {
        SetPosition(pos);
    },
    {this}).ToPtr();
}

frc2::CommandPtr Climber::IdleCommand() {
    return SetPositionCommand(CONSTANTS::CLIMBER::DEFAULT_POS);
};

frc2::CommandPtr Climber::Extend(){
    return SetPositionCommand(CONSTANTS::CLIMBER::EXTEND_POS);
};

frc2::CommandPtr Climber::Climb(){
    return SetPositionCommand(CONSTANTS::CLIMBER::CLIMB_POS);
};

void Climber::SetPosition(units::angle::turn_t pos) {
    m_motor.SetControl(ctre::phoenix6::controls::PositionTorqueCurrentFOC{pos});
};

units::degree_t Climber::GetAngle()
{
  return m_motor.GetPosition().GetValue();
};

