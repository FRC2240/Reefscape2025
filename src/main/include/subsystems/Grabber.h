#pragma once
#include <ctre/phoenix6/TalonFX.hpp>
#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <utility/MotorUtils.h>
#include <TimeOfFlight.h>
#include <frc2/command/Commands.h>
#include <ctre/phoenix6/CANrange.hpp>
#include <units/current.h>

class Grabber : public frc2::SubsystemBase
{
public:
    Grabber();

    frc2::CommandPtr intake(units::ampere_t current);
    frc2::CommandPtr intake_algae(units::ampere_t current);
    frc2::CommandPtr extake(units::ampere_t current);
    frc2::CommandPtr idle();
    frc2::CommandPtr coral_release();
    frc2::CommandPtr coast();

    void InitSendable(wpi::SendableBuilder &builder) override;
    void SetPID();

    bool has_gp();

    pwf::TimeOfFlight Grabber_sensor{CONSTANTS::GRABBER::TOF_ID};

private:
    ctre::phoenix6::hardware::TalonFX m_motor{CONSTANTS::GRABBER::MOTOR_ID};

    CONSTANTS::PidCoeff coeff{CONSTANTS::GRABBER::PID};

    // ctre::phoenix6::hardware::CANrange m_can_range{CONSTANTS::GRABBER::TOF_ID};

    void spin(units::ampere_t current);
};