#include <subsystems/Grabber.h>
#include <iostream>

Grabber::Grabber()
{
    MotorUtils::Motor::LogValues log_values{true, true, true, true, true};
    MotorUtils::Motor right_grabber_motor{&m_right_motor, CONSTANTS::GRABBER::PID, log_values};
    MotorUtils::Motor left_grabber_motor{&m_left_motor, CONSTANTS::GRABBER::PID, log_values};

    AddPID(right_grabber_motor);
    AddPID(left_grabber_motor);
    SetPID();
};

void Grabber::spin(units::turns_per_second_t speed){
    ctre::phoenix6::controls::VelocityVoltage velocity{speed};
    m_right_motor.SetControl(velocity);
    m_left_motor.SetControl(velocity);
};

void Grabber::intake(units::turns_per_second_t speed){
    spin(CONSTANTS::GRABBER::INTAKE_VELOCITY);
    if(units::millimeter_t{Grabber_sensor.GetRange()} < CONSTANTS::GRABBER::DEFAULT_DIST_TOF){
            spin(0_tps);
    };
};

frc2::CommandPtr Grabber::extake(units::turns_per_second_t speed, units::second_t time){
    return frc2::cmd::Run(
        [this, speed] {
            spin(speed);
        }, {this}
    ).WithTimeout(time);
};