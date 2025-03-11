#pragma once
#include "subsystems/Candle.h"

Candle::Candle(std::function<bool()> HasGP)
{
    this->HasGP = HasGP;
    ctre::phoenix::led::CANdleConfiguration config;
    config.stripType = ctre::phoenix::led::LEDStripType::RGB; // GRB?
    config.brightnessScalar = 0.5;
    m_candle.ConfigAllSettings(config);
    m_candle.ClearAnimation(0);
    m_candle.SetLEDs(0, 0, 0);
    m_candle.ConfigLOSBehavior(1, 1000);
};

void Candle::Periodic()
{

    if (frc::DriverStation::GetAlliance() && frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
        teamcolor.r = 255;
        teamcolor.g = 0;
        teamcolor.b = 0;
    }
    else if (frc::DriverStation::GetAlliance() && frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
    {
        teamcolor.r = 0;
        teamcolor.g = 0;
        teamcolor.b = 255;
    }
    m_larson_auto.SetR(teamcolor.r);
    m_larson_auto.SetB(teamcolor.b);
    m_larson_auto.SetG(teamcolor.g);

    /*
    Step 1: Determine desired state
    Step 1a: Determine if a state change occurs
    Stept 2: Switch on state
    */
    if (IsAuto()) // Auto should take priority state
    {
        state = AUTO;
    }
    else if (frc::DriverStation::IsDisabled())
    {

        if ((cycles_in_state / 50.0) < 0.5) // Cycles/50 = seconds
        {
            state = DISABLE_BLINK;
            fmt::println("state is disabled blink");
        }
        else if (frc::DriverStation::IsEStopped())
        {
            state = ERROR;
        }
        else
        {
            state = DISABLED; // Not always blinking. Assume normal disabled then check for time in state
        }
    }
    else if (HasGP())
    {
        wants_r = 0;
        wants_g = 0;
        wants_b = 0;
        state = HASGP;
    }
    else if (wants_r + wants_g + wants_b > 0 && !HasGP()) // Rationale is that getting a GP resets all these to 0
    {
        state = WANTGP;
    }
    else if (frc::DriverStation::GetBatteryVoltage() < 9.0) // Turn LEDs off to conserve voltage under brown
    {
        state = BROWN;
    }
    else if (frc::DriverStation::IsEnabled()) // Must be last because WantGP and HasGP conflict
    {
        state = ENABLED;
    }
    else
    {
        state = ERROR;
    }

    if (prev_state != state && prev_state != DISABLE_BLINK)
    {
        cycles_in_state = 0;
        m_candle.ClearAnimation(0);
    }

    if (cycles_in_state == 0)
    {
        m_candle.ClearAnimation(0);
    }
    cycles_in_state++;
    prev_state = state;

    switch (state)
    {
    case AUTO:
        frc::SmartDashboard::PutString("candlesim", "#8ACE00");
        m_candle.Animate(m_larson_auto);
        break;

    case WANTGP:
        if (cycles_in_state % 2 == 0) // Blink every other frame, consider rasing modulo operand to reduce seizure risk
        {
            m_candle.SetLEDs(wants_r, wants_g, wants_b);
            frc::SmartDashboard::PutString("candlesim", frc::Color(wants_r, wants_b, wants_g).HexString());
        }
        else
        {
            frc::SmartDashboard::PutString("candlesim", "#000000");
            m_candle.SetLEDs(0, 0, 0);
        }

        break;

    case HASGP:
        frc::SmartDashboard::PutString("candlesim", "#00FF00");
        m_candle.SetLEDs(0, 255, 0);
        break;

    case DISABLED:

        frc::SmartDashboard::PutString("candlesim", frc::Color(teamcolor.r, teamcolor.g, teamcolor.b).HexString());
        m_candle.SetLEDs(teamcolor.r, teamcolor.g, teamcolor.b);
        break;

    case DISABLE_BLINK:
        if (cycles_in_state % 10 > 5)
        {
            frc::SmartDashboard::PutString("candlesim", "#FFFF00");
            m_candle.SetLEDs(255, 255, 0);
        }
        else
        {
            frc::SmartDashboard::PutString("candlesim", "#000000");
            m_candle.SetLEDs(0, 0, 0);
        }
        break;

    case ENABLED:
        frc::SmartDashboard::PutString("candlesim", "#FF00FF");
        m_candle.Animate(m_rainbow);
        break;

    case ERROR:
        if (cycles_in_state % 10 > 5)
        {
            frc::SmartDashboard::PutString("candlesim", "#FF0000");
            m_candle.SetLEDs(255, 0, 0);
        }
        else
        {
            frc::SmartDashboard::PutString("candlesim", "#FFFF00");
            m_candle.SetLEDs(255, 255, 0);
        }
        break;

    // Added fuctionality to turn off LEDs on brown
    case BROWN:
        frc::SmartDashboard::PutString("candlesim", "#000000");
        m_candle.SetLEDs(0, 0, 0);
        break;

    default:
        frc::SmartDashboard::PutString("candlesim", "#FFFFFF");
        m_candle.SetLEDs(255, 255, 255);
        break;
    }
};

void Candle::WantGP(int r, int g, int b)
{
    wants_r = r;
    wants_g = g;
    wants_b = b;
};

bool Candle::IsAuto()
{
    return frc::DriverStation::IsAutonomousEnabled();
};

bool Candle::IsRed()
{
    if (frc::DriverStation::GetAlliance())
    {
        if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
        {
            return false;
        }
    }
    return true;
};
