#include <subsystems/Grabber.h>


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
    ctre::phoenix6::controls::VelocityVoltage inverse_velocity{-1 * speed};
    m_left_motor.SetControl(velocity);
    m_right_motor.SetControl(inverse_velocity);
    //unsure if I should do it this way or with the set inverse follow thing
};

frc2::CommandPtr Grabber::intake(units::turns_per_second_t speed){
    return frc2::cmd::Run(
        [this, speed] {
            spin(speed);
        }, {this}
    ).Until([this] -> bool {
        return units::millimeter_t{Grabber_sensor.GetRange()} < CONSTANTS::GRABBER::DEFAULT_DIST_TOF;
    });
};

frc2::CommandPtr Grabber::extake(units::turns_per_second_t speed, units::second_t time){
    return frc2::cmd::Run(
        [this, speed] {
            spin(speed);
        }, {this}
    ).WithTimeout(time);
};