#pragma once
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "ctre/phoenix/led/CANdle.h"
#include "ctre/phoenix/led/RainbowAnimation.h"
#include "ctre/phoenix/led/LarsonAnimation.h"
#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <units/time.h>
#include "Constants.h"

class Candle : public frc2::SubsystemBase {
    public:
        Candle(std::function<bool()> HasGP );
        void Periodic() override;
        void WantGP(int r, int g, int b);

        std::function<bool()> HasGP;
    private:
        bool IsAuto();
        bool IsRed();

        struct Color
        {
            int r,g,b;
        };
        

        enum CANDLE_STATE {
            AUTO,
            WANTGP,
            HASGP,
            DISABLED,
            DISABLE_BLINK,
            ENABLED,
            ERROR,
            BROWN,
                                };

        unsigned long long cycles_in_state = 0;
        // char cycles_in_state = 0;
        Color teamcolor{255,255,255};
        int blink_counter = 0;
        CANDLE_STATE state = ERROR;
        CANDLE_STATE prev_state = ERROR;
        int wants_r = 0;
        int wants_g = 0;
        int wants_b = 0;

        frc::Timer timer;
        ctre::phoenix::led::CANdle m_candle{CONSTANTS::CANDLE::CANDLE_ID};
                                                          // r    g   b  w  spd  num
        ctre::phoenix::led::LarsonAnimation m_larson_auto{255, 255, 0, 0, 0.5, CONSTANTS::CANDLE::NUM_LEDS};
        //                                              brt  spd
        ctre::phoenix::led::RainbowAnimation m_rainbow {0.5, 0.5};
        // ctre::phoenix::led::
};