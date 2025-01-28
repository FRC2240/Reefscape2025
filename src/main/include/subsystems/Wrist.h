#pragma once

#include "utility/MotorUtils.h"
#include "Constants.h"

#include <units/angle.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "frc2/command/CommandPtr.h"
#include <frc2/command/SubsystemBase.h>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>
#include <wpi/sendable/SendableBuilder.h>
#include <iostream>

class Wrist : public frc2::SubsystemBase
{
public:

    Wrist();
    void SetPID();

    void InitSendable(wpi::SendableBuilder& builder) override;

    void set_angle(units::angle::degree_t angle);
    units::degree_t get_angle();

    frc2::CommandPtr set_angle_command(units::degree_t pos);

private:
    ctre::phoenix6::hardware::TalonFX m_motor {CONSTANTS::WRIST::WRIST_ID};
    CONSTANTS::PidCoeff coeff {CONSTANTS::WRIST::PidValue};
};