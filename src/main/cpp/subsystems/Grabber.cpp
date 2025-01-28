#include <subsystems/Grabber.h>

Grabber::Grabber()
{
    //MotorUtils::Motor::LogValues log_values{true, true, true, true, true};
    //MotorUtils::Motor right_grabber_motor{&m_right_motor, CONSTANTS::GRABBER::PID, log_values};
    //MotorUtils::Motor left_grabber_motor{&m_left_motor, CONSTANTS::GRABBER::PID, log_values};

    //AddPID(right_grabber_motor);
    //AddPID(left_grabber_motor);
   // ctre::phoenix6::configs::TalonFXConfiguration m_right_conf{};

    // TODO: verify correctness of this config.
    //SetPID();
    m_right_motor.SetControl(ctre::phoenix6::controls::Follower{m_left_motor.GetDeviceID(), 1});
};

void Grabber::spin(units::turns_per_second_t speed)
{
    ctre::phoenix6::controls::VelocityTorqueCurrentFOC velocity{speed};
    m_left_motor.SetControl(velocity);
};

frc2::CommandPtr Grabber::intake_command(units::turns_per_second_t speed)
{
    return frc2::cmd::Run(
               [this, speed]
               {
                   spin(speed);
               },
               {this})
        .Until([this] -> bool
               { return units::millimeter_t{Grabber_sensor.GetRange()} < CONSTANTS::GRABBER::DEFAULT_DIST_TOF; })
        .AndThen([this]
                 { spin(0_tps); })
        .WithName("Intake");
};

frc2::CommandPtr Grabber::extake_command(units::turns_per_second_t speed, units::second_t time)
{
    return frc2::cmd::Run(
               [this, speed]
               {
                   spin(speed);
               },
               {this})
        .WithTimeout(time)
        .AndThen([this]
                 { spin(0_tps); })
        .WithName("Extaxe");
};