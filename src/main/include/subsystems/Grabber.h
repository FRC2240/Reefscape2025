#pragma once
#include <ctre/phoenix6/TalonFX.hpp>
#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <utility/MotorUtils.h>
#include <TimeOfFlight.h>
#include <frc2/command/Commands.h>
#include <ctre/phoenix6/CANrange.hpp>

class Grabber : public frc2::SubsystemBase
{
public:
    Grabber();

    frc2::CommandPtr intake(units::turns_per_second_t speed);
    frc2::CommandPtr extake();
    frc2::CommandPtr idle();
    /* NO MOTORS, NO FUNCTIONS

    void InitSendable(wpi::SendableBuilder &builder) override;
    void SetPID();
    */

    bool has_gp();

private:
    //ctre::phoenix6::hardware::TalonFX m_right_motor{CONSTANTS::GRABBER::RIGHT_ID};
    //ctre::phoenix6::hardware::TalonFX m_left_motor{CONSTANTS::GRABBER::LEFT_ID};
    pwf::TimeOfFlight Grabber_sensor{CONSTANTS::GRABBER::TOF_ID};
    //ctre::phoenix6::hardware::CANrange m_can_range{CONSTANTS::GRABBER::TOF_ID};

    //void spin(units::turns_per_second_t speed);
};