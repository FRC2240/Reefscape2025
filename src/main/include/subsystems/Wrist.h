#pragma once

#include "utility/MotorUtils.h"
#include "Constants.h"

#include <units/angle.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "frc2/command/CommandPtr.h"
#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

class Wrist : public frc2::SubsystemBase
{
public:
    Wrist();
    void set_angle(units::angle::degree_t angle);
    units::degree_t get_angle();

    void SetPID();
    void InitSendable(wpi::SendableBuilder &builder) override;

    frc2::CommandPtr rezero();
    frc2::CommandPtr set_angle_command(units::degree_t pos);
    frc2::CommandPtr offset_command(units::degree_t amount);

private:
    ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC m_control_req{0_tr};
    ctre::phoenix6::hardware::TalonFX m_motor{CONSTANTS::WRIST::WRIST_ID};

    CONSTANTS::PidCoeff coeff{CONSTANTS::WRIST::PidValue};
};