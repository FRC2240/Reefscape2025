#pragma once

#include "utility/MotorUtils.h"
#include "Constants.h"

#include <units/angle.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "frc2/command/CommandPtr.h"
#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

class Ground : public frc2::SubsystemBase
{
    public:
        Ground();

        void set_angle(ctre::phoenix6::hardware::TalonFX& motor, units::angle::degree_t angle);
        units::degree_t get_angle(ctre::phoenix6::hardware::TalonFX& motor);

        void SetPID(ctre::phoenix6::hardware::TalonFX& motor, CONSTANTS::PidCoeff coeff);

        void InitSendable(wpi::SendableBuilder &builder) override;
    private:
        ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC m_control_req{0_tr};

        ctre::phoenix6::hardware::TalonFX m_ground{CONSTANTS::GROUND::GROUND_ID};
        ctre::phoenix6::hardware::TalonFX m_intake{CONSTANTS::GROUND::INTAKE_ID};
        ctre::phoenix6::hardware::TalonFX m_index{CONSTANTS::GROUND::INDEX_ID};

        CONSTANTS::PidCoeff ground_coeff{CONSTANTS::GROUND::groundPID};
        CONSTANTS::PidCoeff intake_coeff{CONSTANTS::GROUND::intakePID};
        CONSTANTS::PidCoeff index_coeff{CONSTANTS::GROUND::indexPID};
};