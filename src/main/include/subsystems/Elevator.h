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
#include <frc/DataLogManager.h>

class Elevator : public frc2::SubsystemBase
{

public:
    Elevator();

    frc2::CommandPtr set_position_command(units::angle::turn_t pos);
    frc2::CommandPtr idle_command();

    units::angle::turn_t get_position();
    void set_position(units::angle::turn_t pos);
    frc2::CommandPtr offset_command(units::angle::turn_t amount);

    void InitSendable(wpi::SendableBuilder &builder) override;
    void SetPID();

    // moved to public to support selftest
    ctre::phoenix6::hardware::TalonFX m_motor{CONSTANTS::ELEVATOR::LEFT_ID};
    ctre::phoenix6::hardware::TalonFX m_follower_motor{CONSTANTS::ELEVATOR::RIGHT_ID};

private:
    ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC control_req{0_tr};

    CONSTANTS::PidCoeff coeff{CONSTANTS::ELEVATOR::PidValue};
};