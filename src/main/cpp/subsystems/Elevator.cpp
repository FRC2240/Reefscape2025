#pragma once
#include "subsystems/Elevator.h"

Elevator::Elevator()
{
    ctre::phoenix6::configs::TalonFXConfiguration conf{};
    conf.MotionMagic.MotionMagicAcceleration = 250_tr_per_s_sq;
    conf.MotionMagic.MotionMagicCruiseVelocity = 35_tps;

    m_motor.GetConfigurator().Apply(conf);
    SetPID();
    m_follower_motor.SetControl(ctre::phoenix6::controls::Follower(m_motor.GetDeviceID(), true));
}

void Elevator::SetPID()
{
    SetPID(m_motor, coeff);
}

void Elevator::InitSendable(wpi::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("Elevator");
    BuildSender(builder, &coeff);
    // BuildSender(builder, &m_motor);
}

frc2::CommandPtr Elevator::set_position_command(units::angle::turn_t pos)
{
    return frc2::RunCommand([this, pos]
                            {
        units::angle::turn_t position = pos;
        if (position > CONSTANTS::ELEVATOR::TOP_POS) {
            position = CONSTANTS::ELEVATOR::TOP_POS;
        } else if (position < CONSTANTS::ELEVATOR::BOTTOM_POS) {
            position = CONSTANTS::ELEVATOR::BOTTOM_POS;
        }
        frc::SmartDashboard::PutNumber("elv/desired", position.value());
        set_position(position); },
                            {this})
        .Until([this, pos]
               { return CONSTANTS::IN_THRESHOLD<units::angle::turn_t>(get_position(), pos, CONSTANTS::ELEVATOR::POSITION_THRESHOLD); });
}

frc2::CommandPtr Elevator::idle_command()
{
    return set_position_command(CONSTANTS::ELEVATOR::BOTTOM_POS);
};

frc2::CommandPtr Elevator::follow_joystick_command(frc2::CommandXboxController *stick)
{
    return frc2::RunCommand([this, stick]
                            {
        double stickpos = stick->GetLeftY(); // CHANGEME
        if (
            (stickpos < -CONSTANTS::ELEVATOR::DEADBAND_THRESHOLD && get_position() > CONSTANTS::ELEVATOR::BOTTOM_POS)
            || (stickpos > CONSTANTS::ELEVATOR::DEADBAND_THRESHOLD && get_position() < CONSTANTS::ELEVATOR::TOP_POS)
        ) {
            // m_motor.SetControl(control_req.WithVelocity(CONSTANTS::ELEVATOR::JOYSTICK_SPEED * stickpos));
        } else {
            // m_motor.SetControl(control_req.WithVelocity(0_tps));
        }; },
                            {this})
        .ToPtr();
};

units::angle::turn_t Elevator::get_position()
{
    return m_motor.GetPosition().GetValue();
};

void Elevator::set_position(units::angle::turn_t pos)
{
    m_motor.SetControl(control_req.WithPosition(pos));
};

void Elevator::SetPID(ctre::phoenix6::hardware::TalonFX &motor, CONSTANTS::PidCoeff coeff)
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

void Elevator::BuildSender(wpi::SendableBuilder &builder, CONSTANTS::PidCoeff *coeff)
{

    builder.AddDoubleProperty(
        "current",
        [this]
        { return m_motor.GetStatorCurrent().GetValueAsDouble(); },
        nullptr);

    builder.AddDoubleProperty(
        "vel",
        [this]
        { return m_motor.GetVelocity().GetValueAsDouble(); },
        nullptr);
    builder.AddDoubleProperty(
        "pos",
        [this]
        { return m_motor.GetPosition().GetValueAsDouble(); },
        nullptr);
    builder.AddDoubleProperty(
        "PID/P",
        [coeff]
        { return coeff->GetP(); },
        [coeff](double val)
        { coeff->SetP(val); });
    builder.AddDoubleProperty(
        "PID/I",
        [coeff]
        { return coeff->GetI(); },
        [coeff](double val)
        { coeff->SetI(val); });
    builder.AddDoubleProperty(
        "PID/D",
        [coeff]
        { return coeff->GetD(); },
        [coeff](double val)
        { coeff->SetD(val); });
    builder.AddDoubleProperty(
        "PID/S",
        [coeff]
        { return coeff->GetS(); },
        [coeff](double val)
        { coeff->SetS(val); });
    builder.AddDoubleProperty(
        "PID/G",
        [coeff]
        { return coeff->GetG(); },
        [coeff](double val)
        { coeff->SetG(val); });
    builder.AddDoubleProperty(
        "PID/Min",
        [coeff]
        { return coeff->GetMin(); },
        [coeff](double val)
        { coeff->SetMin(val); });
    builder.AddDoubleProperty(
        "PID/Max",
        [coeff]
        { return coeff->GetMax(); },
        [coeff](double val)
        { coeff->SetMax(val); });
}
