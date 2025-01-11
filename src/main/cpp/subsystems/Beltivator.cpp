#pragma once
#include "subsystems/Beltivator.h"


Beltivator::Beltivator() {
    MotorUtils::Motor::LogValues logValues {true, true, true};
    MotorUtils::Motor beltivatorMotor{&m_motor, CONSTANTS::BELTIVATOR::PidValue, logValues};
    AddPID(beltivatorMotor);
    SetPID();
    
    //ctre::phoenix6::configs::TalonFXConfiguration beltivator_config{};
    //m_motor.GetConfigurator().Apply(beltivator_config)
}


frc2::CommandPtr Beltivator::setPositionCommand(units::angle::turn_t pos) {
    return frc2::RunCommand([this, pos] {
        setPosition(pos);
    },
    {this}).ToPtr();
}


frc2::CommandPtr Beltivator::idleCommand() {
    return setPositionCommand(CONSTANTS::BELTIVATOR::BOTTOM_POS);
};


frc2::CommandPtr Beltivator::followJoystickCommand(frc2::CommandXboxController* stick) {
    return frc2::RunCommand([this, stick] {
        //double stickpos = stick.GetLeftY() // CHANGEME
    },
    {this}).ToPtr();
};


units::angle::turn_t Beltivator::getPosition() {
    return m_motor.GetPosition().GetValue();
};


void Beltivator::setPosition(units::angle::turn_t pos) {
    m_motor.SetControl(ctre::phoenix6::controls::PositionVoltage{pos});
};