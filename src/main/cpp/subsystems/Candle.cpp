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
    m_candle.ConfigLOSBehavior(1,1000);
};

void Candle::Periodic()
{
    if (frc::DriverStation::GetAlliance() && frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kRed){
        teamcolor.r=255;
        teamcolor.g =0;
        teamcolor.b =0;
    }
    else if (frc::DriverStation::GetAlliance() && frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue){
        teamcolor.r = 0;
        teamcolor.g = 0;
        teamcolor.b = 255;
    }
    m_larson_auto.SetR(teamcolor.r);
    m_larson_auto.SetB(teamcolor.b);
 
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
        else {
        state = DISABLED; // Not always blinking. Assume normal disabled then check for time in state
        }
    }
    
    else if (HasGP())
    {
        wants_r = 0;
        wants_g = 0;
        wants_b = 0;
        state = HASGP;
    }else if (wants_r + wants_g + wants_b > 0 && !HasGP())  // Rationale is that getting a GP resets all these to 0
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

    if (prev_state != state && prev_state!= DISABLE_BLINK)
    {
        cycles_in_state = 0;
        m_candle.ClearAnimation(0);
    }
    cycles_in_state++;
    prev_state = state;

    if (cycles_in_state == 0)
    {
        m_candle.ClearAnimation(0);
    }
    switch (state)
    {
    case AUTO:

        m_candle.Animate(m_larson_auto);
        break;

    case WANTGP:
        if (cycles_in_state % 2 == 0) // Blink every other frame, consider rasing modulo operand to reduce seizure risk
        {
            m_candle.SetLEDs(wants_r, wants_g, wants_b);
        }
        else
        {
            m_candle.SetLEDs(0, 0, 0);
        }

        break;

    case HASGP:
        m_candle.SetLEDs(0, 255, 0);
        break;

    case DISABLED:
        
            m_candle.SetLEDs(teamcolor.r, teamcolor.g, teamcolor.b);
        break;

    case DISABLE_BLINK:
        if (cycles_in_state % 10 > 5)
        {
            m_candle.SetLEDs(255, 255, 0);
            fmt::println("db on");
        }
        else
        {
            m_candle.SetLEDs(0, 0, 0);
            fmt::println("db off");
        }
        break;

    case ENABLED:
        m_candle.Animate(m_rainbow);
        break;

    case ERROR:
        // Fallthrough intented

    //Added fuctionality to turn off LEDs on brown
    case BROWN:
        m_candle.SetLEDs(0,0,0);
        break;

    default:
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
