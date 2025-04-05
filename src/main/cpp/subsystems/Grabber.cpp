// This code is self doccumenting.
#include <subsystems/Grabber.h>

Grabber::Grabber()
{
    // MotorUtils::Motor::LogValues log_values{true, true, true, true, true};
    // MotorUtils::Motor right_grabber_motor{&m_right_motor, CONSTANTS::GRABBER::PID, log_values};
    // MotorUtils::Motor left_grabber_motor{&m_left_motor, CONSTANTS::GRABBER::PID, log_values};

    // AddPID(right_grabber_motor);
    // AddPID(left_grabber_motor);
    ctre::phoenix6::configs::TalonFXConfiguration conf{};

    conf.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    conf.Slot0.kS = 1;
    conf.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

    SetPID();

    m_motor.GetConfigurator().Apply(conf);

    // ctre::phoenix6::configs::CANrangeConfiguration can_range_conf{};
    // can_range_conf.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
    // can_range_conf.ProximityParams.ProximityThreshold = 4_in;
    // m_can_range.GetConfigurator().Apply(can_range_conf);
};

void Grabber::InitSendable(wpi::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("Grabber");
    MotorUtils::BuildSender(builder, &m_motor);
    MotorUtils::BuildSender(builder, &coeff);
};

void Grabber::SetPID()
{
    MotorUtils::SetPID(m_motor, coeff); // is necesary with following?
};

bool Grabber::has_gp()
{
    return (units::millimeter_t{Grabber_sensor.GetRange()} < CONSTANTS::GRABBER::DEFAULT_DIST_TOF && Grabber_sensor.GetStatus() == pwf::TimeOfFlight::Status::kValid);
};

void Grabber::spin(units::ampere_t current)
{
    ctre::phoenix6::controls::TorqueCurrentFOC req{current};
    m_motor.SetControl(req);
};

frc2::CommandPtr Grabber::idle()
{
    return frc2::cmd::Run([this]
                          { spin(0_A); },
                          {this})
        .WithName("Idle");
};

frc2::CommandPtr Grabber::extake(units::ampere_t speed)
{
    return frc2::cmd::Run(
               [this, speed]
               {
                   spin(speed);
               },
               {this})
        .WithTimeout(CONSTANTS::GRABBER::EXTAKE_TIME)
        .AndThen([this]
                 { spin(0_A); })
        .WithName("Extake");
};

frc2::CommandPtr Grabber::intake(units::ampere_t speed)
{
    return frc2::cmd::Run(
               [this, speed]
               {
                   spin(speed);
               },
               {this})
        .Until([this]
               { return has_gp(); })
        .WithName("Intake");
};

frc2::CommandPtr Grabber::intake_algae(units::ampere_t speed)
{
    return frc2::cmd::Run(
               [this, speed]
               {
                   spin(speed);
               },
               {this})
        .WithName("Intake_algae");
};

frc2::CommandPtr Grabber::coral_release()
{
    return frc2::cmd::Run([this]
                          {
                ctre::phoenix6::controls::VelocityTorqueCurrentFOC req{CONSTANTS::GRABBER::CORAL_RELEASE_VELOCITY};
                m_motor.SetControl(req); })
        .WithName("Coral_release");
}

frc2::CommandPtr Grabber::coast()
{
    return frc2::cmd::Run(
        [this]
        {
            m_motor.SetControl(ctre::phoenix6::controls::CoastOut{});
        },
        {this});
}
