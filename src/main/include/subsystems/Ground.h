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
        units::degree_t offset = 0_tr;

        Ground();

        void SetPID(ctre::phoenix6::hardware::TalonFX& m_motor, CONSTANTS::PidCoeff coeff);

        void InitSendable(wpi::SendableBuilder &builder) override;

        void set_angle(ctre::phoenix6::hardware::TalonFX& m_motor, units::angle::degree_t angle);
        units::degree_t get_angle(ctre::phoenix6::hardware::TalonFX& m_motor);

        void intake();
        void eject();

        bool hasGP(); // Needs a sensor input to work currently pulling from CONSTANTS::GROUND::test_sensor

        void Periodic();

        frc2::CommandPtr offset_command(units::degree_t amount);
        frc2::CommandPtr intake_command();
        frc2::CommandPtr eject_command();

    private:

        ctre::phoenix6::controls::PositionTorqueCurrentFOC m_control_req{0_tr};

        ctre::phoenix6::hardware::TalonFX m_ground{CONSTANTS::GROUND::GROUND_ID};
        ctre::phoenix6::hardware::TalonFX m_intake{CONSTANTS::GROUND::INTAKE_ID};
        ctre::phoenix6::hardware::TalonFX m_index{CONSTANTS::GROUND::INDEX_ID};

        CONSTANTS::PidCoeff ground_coeff{CONSTANTS::GROUND::groundPID};
        CONSTANTS::PidCoeff intake_coeff{CONSTANTS::GROUND::intakePID};
        CONSTANTS::PidCoeff index_coeff{CONSTANTS::GROUND::indexPID};
};