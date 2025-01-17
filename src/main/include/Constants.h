#pragma once

#include <frc/DriverStation.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <iostream>
#include <numbers>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <vector>
#include <units/current.h>

// #define COLFAX_BOT

#define SABERTOOTH
//  When using the second robot, uncomment the above line

namespace CONSTANTS
{

    constexpr units::ampere_t DEFAULT_CURRENT_LIMIT = 60_A;

    // An additive threshold (+/- value) that checks if 2 values (target & source)
    // are within a range A template so it can be used with units. Call it by:
    // CONSTANTS::IN_THRESHOLD<type>(x,y,z)
    template <typename T>
    static bool IN_THRESHOLD(T source, T target, T range)
    {
        return (source >= target - range && source <= target + range);
    }

    struct PidCoeff
    {
        double kP = 0; // Proportion
        double kI = 0; // Integral
        double kD = 0; // Derivative
        double kS = 0; // Static gain
        double kG = 0; // Gravity gain

        double min = -1; // Minimum output for control loop
        double max = 1;  // Maximum output for control loop
    };

    namespace CORAL
    {

        static const std::string LIMELIGHT_ID = "limelight-aft";
        constexpr units::meters_per_second_t APPROACH_SPEED = 0.25_mps;

    }

    namespace GRABBER
    {
        constexpr int LEFT_ID = 29;  // changeme
        constexpr int RIGHT_ID = 30; // changeme
        // Configuration page: http://10.22.40.2:5812
        constexpr int TOF_ID = 32; // changeme

        static CONSTANTS::PidCoeff PID = {1};

        // intake
        constexpr units::turns_per_second_t INTAKE_VELOCITY = 1_tps; // TBD
        constexpr units::millimeter_t DEFAULT_DIST_TOF = 20_mm;      // TBD

        // extake
        constexpr units::turns_per_second_t EXTAKE_VELOCITY = 1_tps; // TBD
        constexpr units::second_t EXTAKE_TIME = 1_s;                 // TBD
    }

    namespace ELEVATOR
    {
        constexpr units::angle::turn_t BOTTOM_POS = 0_tr;

        constexpr units::angle::turn_t TOP_POS = 1_tr;
        constexpr int ELEVATOR_ID = 50; // CHANGEME
        constexpr double DEADBAND_THRESHOLD = 0.1;
        static const PidCoeff PidValue = {1};
        constexpr units::angular_velocity::turns_per_second_t JOYSTICK_SPEED = 1_tps;
        namespace PRESETS
        {
            constexpr units::angle::turn_t BOTTOM = BOTTOM_POS;
            constexpr units::angle::turn_t TOP = TOP_POS;
        }
    }

    namespace WRIST
    {
        constexpr int WRIST_ID = 29;
        constexpr units::angle::turn_t DEFAULT_POSITION = 0_tr;
        // This is the default PID values for the wrist motor
        static const PidCoeff PidValue = {1};

        constexpr units::angle::degree_t POSITION_THRESHOLD = 5_deg;
    };

    namespace VISION
    {
        static const auto LEFT_CAMERA_A_TF = frc::Transform3d{
            0.151_m, 0.319_m, 0.578_m, frc::Rotation3d(180_deg, -10_deg, 90_deg)};
        static const auto RIGHT_CAMERA_A_TF = frc::Transform3d{
            0.151_m, -0.319_m, 0.578_m, frc::Rotation3d(180_deg, -10_deg, -90_deg)};

    } // namespace VISION

    namespace CLIMBER
    {
        constexpr int CLIMBER_ID = 29;

        constexpr units::angle::turn_t DEFAULT_POS = 0_tr;
        constexpr units::angle::turn_t EXTEND_POS = 100_tr;
        constexpr units::angle::turn_t CLIMB_POS = 50_tr;

        static const PidCoeff PidValue = {1};

    } // namespace CLIMBER

    namespace CANDLE
    {
        constexpr int CANDLE_ID = 10;
        constexpr int NUM_LEDS = 39;
    } // namespace CANDLE

    namespace SHOOTER
    {
        constexpr int LEFT_ID = 2;
        constexpr int RIGHT_ID = 5;
        constexpr int ANGLE_ID = 6;
        constexpr int ANGLE2_ID = 2;
        constexpr int CANCODER_ID = 13; // CHANGEME
        constexpr std::pair<units::turn_t, units::turn_t> FENDER_RANGE = {0_tr, 1_tr};
        constexpr double ANGLE_RATIO = 1; // CHANGEME

#ifdef SABERTOOTH // Main robot config
        constexpr units::turn_t FENDER_ANGLE = 11.5_tr;
        constexpr units::turn_t AMP_ANGLE = 11_tr;
        constexpr units::turns_per_second_t AMP_VELOCTITY = -6.5_tps;
        constexpr units::turn_t REST_ANGLE = 0.242_tr;
        constexpr units::turns_per_second_t SHOOTER_VELOCITY = 60_tps;
#endif
#ifndef SABERTOOTH
        constexpr units::turn_t REST_ANGLE = -0.5_tr;
        constexpr units::turn_t FENDER_ANGLE = -11_tr;
        constexpr units::turn_t AMP_ANGLE = -10_tr;
        constexpr units::turns_per_second_t SHOOTER_VELOCITY = 80_tps;
#endif

        constexpr int BELT_ID = 7;
        constexpr units::turns_per_second_t LEFT_VELOCITY{10};  // CHANGEME;
        constexpr units::turns_per_second_t RIGHT_VELOCITY{10}; // CHANGEME;

    } // namespace SHOOTER

    namespace DRIVE
    {
        constexpr units::second_t BRAKE_TIME = 10_s;
        constexpr units::meters_per_second_t ROBOT_MAX_SPEED = 23.533_fps;
        constexpr units::radians_per_second_t ROBOT_MAX_ANGULAR_SPEED{std::numbers::pi *
                                                                      0.5};
        constexpr units::meters_per_second_t TELEOP_MAX_SPEED = ROBOT_MAX_SPEED;
        constexpr units::radians_per_second_t TELEOP_MAX_ANGULAR_SPEED{
            std::numbers::pi * 0.5};
        constexpr units::meters_per_second_t TRAJ_MAX_SPEED = ROBOT_MAX_SPEED;
        constexpr units::acceleration::meters_per_second_squared_t
            TRAJ_MAX_ACCELERATION = TRAJ_MAX_SPEED / 0.5_s;
        constexpr units::radians_per_second_t TRAJ_MAX_ANGULAR_SPEED =
            CONSTANTS::DRIVE::ROBOT_MAX_ANGULAR_SPEED;
        constexpr units::radians_per_second_squared_t TRAJ_MAX_ANGULAR_ACCELERATION{
            std::numbers::pi};
        static constexpr auto WHEEL_CIRCUMFERENCE = 12.11_in / 1.0_tr;
        // static constexpr auto WHEEL_CIRCUMFERENCE = 11.992_in / 1.0_tr;
        constexpr int GYRO_ID = 48; // CHANGEME

        namespace CONFIG
        {
            struct ModuleConfig
            {
                int driver;
                int azimuth;
                int cancoder;
                units::turn_t offset;
            };
#ifdef COLFAX_BOT
#pragma message("Using Colfax bot")
            constexpr ModuleConfig FL{60, 61, 14, -0.279_tr};
            constexpr ModuleConfig FR{50, 51, 13,
                                      -0.182_tr}; // Was not set becouse no cancoder
            constexpr ModuleConfig BL{30, 31, 11, 0.173_tr};
            constexpr ModuleConfig BR{40, 41, 12, -0.445_tr};
#endif // COLFAX_BOT

#ifndef SABERTOOTH
#pragma message("First Robot Config active")
            /* -------------------------------------------------------------------------- */
            /*                     BEGIN FIRST ROBOT CONFIGURATION                        */
            /* -------------------------------------------------------------------------- */

            constexpr ModuleConfig FL{60, 61, 14, 0.311_tr};
            constexpr ModuleConfig FR{50, 51, 13, 0.281_tr};
            constexpr ModuleConfig BL{30, 31, 11, 0.175_tr};
            constexpr ModuleConfig BR{40, 41, 12, -0.066_tr};

/* -------------------------------------------------------------------------- */
/*                        END FIRST ROBOT CONFIGURATION                       */
/* -------------------------------------------------------------------------- */
#endif // SABERTOOTH

#ifdef SABERTOOTH
#pragma message("Second Robot Config active")
            /* -------------------------------------------------------------------------- */
            /*                       BEGIN SECOND ROBOT CONFIGUATION                      */
            /* -------------------------------------------------------------------------- */
            constexpr ModuleConfig FL{60, 61, 14, -0.413_tr};
            constexpr ModuleConfig FR{50, 51, 13, -0.182_tr}; // old value 0.32_tr - 0.5_tr
            constexpr ModuleConfig BL{30, 31, 11, 0.317_tr};
            constexpr ModuleConfig BR{40, 41, 12, -0.254_tr}; // old vaule -0.253_tr

/* -------------------------------------------------------------------------- */
/*                        END SECOND ROBOT CONFIGUATION                       */
/* -------------------------------------------------------------------------- */
#endif // SABERTOOTH
        } // namespace CONFIG
    } // namespace DRIVE
} // namespace CONSTANTS
