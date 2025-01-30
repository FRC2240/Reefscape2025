#pragma once
#include "subsystems/Elevator.h"


Elevator::Elevator() {
    //erikSetPID();
    
    //ctre::phoenix6::configs::TalonFXConfiguration Elevator_config{};
    //m_motor.GetConfigurator().Apply(Elevator_config)
}

void Elevator::SetPID() {
     MotorUtils::SetPID(m_motor, coeff);
}

void Elevator::InitSendable(wpi::SendableBuilder &builder) {
    builder.SetSmartDashboardType("Elevator");
    MotorUtils::BuildSender(builder, &coeff);
    MotorUtils::BuildSender(builder, &m_motor);
}

frc2::CommandPtr Elevator::set_position_command(units::angle::turn_t pos) {
    return frc2::RunCommand([this, pos] {
        units::angle::turn_t position = pos;
        if (position > CONSTANTS::ELEVATOR::TOP_POS) {
            position = CONSTANTS::ELEVATOR::TOP_POS;
        } else if (position < CONSTANTS::ELEVATOR::BOTTOM_POS) {
            position = CONSTANTS::ELEVATOR::BOTTOM_POS;
        }
        set_position(position);
    },
    {this}).ToPtr();
}


frc2::CommandPtr Elevator::idle_command() {
    return set_position_command(CONSTANTS::ELEVATOR::BOTTOM_POS);
};


frc2::CommandPtr Elevator::follow_joystick_command(frc2::CommandXboxController* stick) {
    return frc2::RunCommand([this, stick] {
        double stickpos = stick->GetLeftY(); // CHANGEME
        if (
            (stickpos < -CONSTANTS::ELEVATOR::DEADBAND_THRESHOLD && get_position() > CONSTANTS::ELEVATOR::BOTTOM_POS)
            || (stickpos > CONSTANTS::ELEVATOR::DEADBAND_THRESHOLD && get_position() < CONSTANTS::ELEVATOR::TOP_POS)
        ) {
            ctre::phoenix6::controls::VelocityTorqueCurrentFOC req{CONSTANTS::ELEVATOR::JOYSTICK_SPEED * stickpos};
            m_motor.SetControl(req);
        } else {
            m_motor.SetControl(ctre::phoenix6::controls::VelocityTorqueCurrentFOC{0_tps});
        };
    },
    {this}).ToPtr();
};


units::angle::turn_t Elevator::get_position() {
    return m_motor.GetPosition().GetValue();
};


void Elevator::set_position(units::angle::turn_t pos) {
    m_motor.SetControl(ctre::phoenix6::controls::PositionVoltage{pos});
};