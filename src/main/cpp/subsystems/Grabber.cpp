#include <subsystems/Grabber.h>

Grabber::Grabber()
{
    /* GRABBER DOES NOT HAVE MOTORS
    MotorUtils::Motor::LogValues log_values{true, true, true, true, true};
    MotorUtils::Motor right_grabber_motor{&m_right_motor, CONSTANTS::GRABBER::PID, log_values};
    MotorUtils::Motor left_grabber_motor{&m_left_motor, CONSTANTS::GRABBER::PID, log_values};

    AddPID(right_grabber_motor);
    AddPID(left_grabber_motor);
    ctre::phoenix6::configs::TalonFXConfiguration m_right_conf{};

    // TODO: verify correctness of this config.
    SetPID();
    m_right_motor.SetControl(ctre::phoenix6::controls::Follower{m_left_motor.GetDeviceID(), 1});
    */

    //ctre::phoenix6::configs::CANrangeConfiguration can_range_conf{};
    //can_range_conf.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
    //can_range_conf.ProximityParams.ProximityThreshold = 4_in;
    //m_can_range.GetConfigurator().Apply(can_range_conf);
    // can sensor using fusion
};

bool Grabber::has_gp()
{
    return units::millimeter_t{Grabber_sensor.GetRange()} < CONSTANTS::GRABBER::DEFAULT_DIST_TOF;
};

/*
void Grabber::spin(units::turns_per_second_t speed)
{
    ctre::phoenix6::controls::VelocityTorqueCurrentFOC velocity{speed};
    m_left_motor.SetControl(velocity);
};

frc2::CommandPtr Grabber::idle()
{
    return frc2::cmd::Run([this]
                          { spin(0_tps); },
                          {this})
        .WithName("Idle");
};

frc2::CommandPtr Grabber::extake()
{
    return frc2::cmd::Run(
               [this]
               {
                   spin(CONSTANTS::GRABBER::EXTAKE_VELOCITY);
               },
               {this})
        .WithTimeout(CONSTANTS::GRABBER::EXTAKE_TIME)
        .AndThen([this]
                 { spin(0_tps); })
        .WithName("Extaxe");
};

frc2::CommandPtr Grabber::intake(units::turns_per_second_t speed)
{
    return frc2::cmd::Run(
               [this, speed]
               {
                   spin(speed);
               },
               {this})
        .WithName("Intake");
};
*/