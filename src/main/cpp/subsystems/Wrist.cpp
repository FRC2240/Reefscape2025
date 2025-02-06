#include "subsystems/Wrist.h"

#include "frc2/command/RunCommand.h"

Wrist::Wrist()
{
    // MotorUtils::Motor::LogValues logValues{true, true, true};
    // MotorUtils::Motor wristMotor{&m_motor, CONSTANTS::WRIST::PidValue, logValues};
    SetPID();

    m_motor.SetPosition(CONSTANTS::WRIST::DEFAULT_POSITION);
}

void Wrist::SetPID(ctre::phoenix6::hardware::TalonFX &motor, CONSTANTS::PidCoeff coeff)
{
    ctre::phoenix6::configs::TalonFXConfiguration config{};
    motor.GetConfigurator().Refresh(config);
    config.Slot0.kS = coeff.kS;
    config.Slot0.kP = coeff.kP;
    config.Slot0.kI = coeff.kI;
    config.Slot0.kD = coeff.kD;
    config.CurrentLimits.StatorCurrentLimit = CONSTANTS::DEFAULT_CURRENT_LIMIT;
    config.Audio.BeepOnConfig = true;
    config.Audio.BeepOnBoot = true;

    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int j = 0; j < MAX_CONFIG_APPLY_ATTEMPTS; j++)
    {
        status = motor.GetConfigurator().Apply(config);
        if (status.IsOK())
            break;
    }
    if (!status.IsOK())
    {
        std::string errString = status.GetName();
        frc::DataLogManager::Log("Could not apply configs, error code: " + errString);
    }
}
void Wrist::InitSendable(wpi::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("Wrist");
    MotorUtils::BuildSender(builder, &coeff);
}

void Wrist::SetPID()
{
    SetPID(m_motor, coeff);
}

void Wrist::set_angle(units::angle::degree_t angle)
{
    m_motor.SetControl(m_control_req.WithPosition(angle));
}

units::degree_t Wrist::get_angle()
{
    return m_motor.GetPosition().GetValue();
}

frc2::CommandPtr Wrist::rezero()
{
    return frc2::RunCommand([this]
                            { m_motor.SetControl(ctre::phoenix6::controls::VoltageOut{-3_V}); },
                            {this})
        .Until([this]
               { return m_motor.GetVelocity().GetValue() < 1.5_tps; })
        .WithName("Rezero Wrist")
        .AndThen(frc2::cmd::RunOnce([this]
                                    { m_motor.SetPosition(0_tr);
                                          m_motor.SetControl(ctre::phoenix6::controls::VoltageOut{0_V}); },
                                    {this}));
}

frc2::CommandPtr Wrist::set_angle_command(units::degree_t pos)
{
    return frc2::RunCommand([this, pos]
                            {
            frc::SmartDashboard::PutNumber("Wrist Setpoint", pos.value());
            frc::SmartDashboard::PutNumber("Wrist position", m_motor.GetPosition().GetValueAsDouble());
            set_angle(pos); },
                            {this})
        .WithName("Set Wrist Angle")
        .Until([this, pos]
               { return CONSTANTS::IN_THRESHOLD<units::angle::degree_t>(get_angle(), pos, CONSTANTS::WRIST::POSITION_THRESHOLD); });
}
