#pragma once
#include "subsystems/Elevator.h"


Elevator::Elevator() {
    MotorUtils::Motor::LogValues logValues {true, true, true};
    MotorUtils::Motor ElevatorMotor{&m_motor, CONSTANTS::ELEVATOR::PidValue, logValues};
    AddPID(ElevatorMotor);
    SetPID();
    
    //ctre::phoenix6::configs::TalonFXConfiguration Elevator_config{};
    //m_motor.GetConfigurator().Apply(Elevator_config)
}


frc2::CommandPtr Elevator::setPositionCommand(units::angle::turn_t pos) {
    return frc2::RunCommand([this, pos] {
        setPosition(pos);
    },
    {this}).ToPtr();
}


frc2::CommandPtr Elevator::idleCommand() {
    return setPositionCommand(CONSTANTS::ELEVATOR::BOTTOM_POS);
};


frc2::CommandPtr Elevator::followJoystickCommand(frc2::CommandXboxController* stick) {
    return frc2::RunCommand([this, stick] {
        double stickpos = stick->GetLeftY(); // CHANGEME
        if (
            (stickpos < -CONSTANTS::ELEVATOR::DEADBAND_THRESHOLD && getPosition() < CONSTANTS::ELEVATOR::BOTTOM_POS)
            || (stickpos > CONSTANTS::ELEVATOR::DEADBAND_THRESHOLD && getPosition() > CONSTANTS::ELEVATOR::TOP_POS)
        ) {
            ctre::phoenix6::controls::VelocityTorqueCurrentFOC req{CONSTANTS::ELEVATOR::JOYSTICK_SPEED * stickpos};
            m_motor.SetControl(req);
        } else {
            m_motor.SetControl(ctre::phoenix6::controls::VelocityTorqueCurrentFOC{0_tps});
        };
    },
    {this}).ToPtr();
};


units::angle::turn_t Elevator::getPosition() {
    return m_motor.GetPosition().GetValue();
};


void Elevator::setPosition(units::angle::turn_t pos) {
    m_motor.SetControl(ctre::phoenix6::controls::PositionVoltage{pos});
};