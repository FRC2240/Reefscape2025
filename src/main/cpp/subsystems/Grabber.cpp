#include <subsystems/Grabber.h>
#include <iostream>

Grabber::Grabber(){

    MotorUtils::Motor::LogValues log_values{true, true, true, true, true}; 
    MotorUtils::Motor right_grabber_motor{&m_right_motor, CONSTANTS::GRABBER::PidValue, log_values};
    MotorUtils::Motor left_grabber_motor{&m_left_motor, CONSTANTS::GRABBER::PidValue, log_values};

};
