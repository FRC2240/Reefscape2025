#pragma once

#include "Constants.h"
#include "utility/BetterSubsystemBase.h"
#include "utility/MotorUtils.h"

#include <units/angle.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "frc2/command/CommandPtr.h"
//#include <frc2/command/button/CommandXboxController.h> (Might be used later)
#include <frc2/command/RunCommand.h>

class Climber : public BetterSubsystemBase {
    public:
        Climber();

        frc2::CommandPtr SetPositionCommand(units::angle::turn_t pos);
        frc2::CommandPtr IdleCommand();
        frc2::CommandPtr Climb();
        frc2::CommandPtr Extend();

        //frc2::CommandPtr rotate();
        units::degree_t GetAngle();

        void SetPosition(units::angle::turn_t pos);
    private:
        ctre::phoenix6::hardware::TalonFX m_motor {CONSTANTS::CLIMBER::CLIMBER_ID};
};