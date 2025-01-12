#pragma once

#include "Constants.h"
#include "utility/BetterSubsystemBase.h"
#include "utility/MotorUtils.h"

#include <units/angle.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "frc2/command/CommandPtr.h"
//#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/RunCommand.h>

class Climber : public BetterSubsystemBase {
    public:
        Climber();

        frc2::CommandPtr setPositionCommand(units::angle::turn_t pos);
        frc2::CommandPtr Idlecommand();
        frc2::CommandPtr Climb();
        frc2::CommandPtr Extend();

        //frc2::CommandPtr rotate();
        units::degree_t get_angle();

        void setPosition(units::angle::turn_t pos);
    private:
        ctre::phoenix6::hardware::TalonFX m_motor {CONSTANTS::CLIMBER::CLIMBER_ID};
};