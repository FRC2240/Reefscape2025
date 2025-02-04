#pragma once

#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include "utility/MotorUtils.h"

#include <units/angle.h>
#include <units/angular_velocity.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "frc2/command/CommandPtr.h"
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/RunCommand.h>

class Elevator : public frc2::SubsystemBase
{

public:
    Elevator();

    frc2::CommandPtr set_position_command(units::angle::turn_t pos);
    frc2::CommandPtr idle_command();
    frc2::CommandPtr follow_joystick_command(frc2::CommandXboxController *stick);

    units::angle::turn_t get_position();
    void set_position(units::angle::turn_t pos);

    void InitSendable(wpi::SendableBuilder& builder) override;
    void BuildSender(wpi::SendableBuilder &builder, CONSTANTS::PidCoeff *coeff)
    void SetPID(ctre::phoenix6::hardware::TalonFX &motor, CONSTANTS::PidCoeff coeff)

private:
    // ctre::phoenix6::controls::DynamicMotionMagicTorqueCurrentFOC control_req{5_tr, 5_tr / 1_s, 5_tr / (1_s * 1_s), 5_tr / (1_s * 1_s * 1_s)};
    ctre::phoenix6::controls::PositionTorqueCurrentFOC control_req{0_tr};
    ctre::phoenix6::hardware::TalonFX m_motor{CONSTANTS::ELEVATOR::LEFT_ID};
    ctre::phoenix6::hardware::TalonFX m_follower_motor{CONSTANTS::ELEVATOR::RIGHT_ID};
};