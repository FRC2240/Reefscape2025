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
// #define COLFAX_BOT

 #define BETABOT
//  When using the second robot, uncomment the above line

// #define MOD_AMP
// When amp module is installed, uncomment the above line

// #define MOD_TRAP
//  When trap module is installed, uncomment the above line

// #define MOD_BUDDY
//  When buddy climber is installed, uncomment the above line

namespace CONSTANTS {

// An additive threshold (+/- value) that checks if 2 values (target & source)
// are within a range A template so it can be used with units. Call it by:
// CONSTANTS::IN_THRESHOLD<type>(x,y,z)
template <typename T> static bool IN_THRESHOLD(T source, T target, T range) {
  return (source >= target - range && source <= target + range);
}

  namespace CORAL {

      static const std::string LIMELIGHT_ID = "limelight-aft";
      constexpr units::meters_per_second_t APPROACH_SPEED = 0.25_mps; 

  }
  
namespace INTAKE {
#ifdef BETABOT
constexpr auto DELAY = 0.15_s;
constexpr units::turn_t UP_POSITION = 0.6_tr;
constexpr units::turn_t DOWN_POSITION = 7.5_tr;
constexpr int INTAKE_VOLTAGE = -12;
#endif
#ifndef BETABOT
constexpr auto DELAY = 0.35_s;
constexpr units::turn_t UP_POSITION = 0.8_tr;
constexpr units::turn_t DOWN_POSITION = 8_tr;
constexpr int INTAKE_VOLTAGE = -12;
#endif
constexpr double LOADED_DIST = 300;
constexpr double LOWER_LOADED_DIST = 350;
constexpr int TOF_ID = 34;
constexpr int LOWER_TOF_ID = 33;
constexpr int BELT_ID = 4;
constexpr int ANGLE_ID = 3;
constexpr units::degree_t AUTO_PICKUP_THRESHOLD = 15_deg;
constexpr units::turn_t BRACE_POSITION = 2.33_tr;
constexpr units::turn_t ROTATION_THRESHOLD = 5_tr;
} // namespace INTAKE
namespace VISION {
static const auto LEFT_CAMERA_A_TF = frc::Transform3d{
    0.151_m, 0.319_m, 0.578_m, frc::Rotation3d(180_deg, -10_deg, 90_deg)};
static const auto RIGHT_CAMERA_A_TF = frc::Transform3d{
    0.151_m, -0.319_m, 0.578_m, frc::Rotation3d(180_deg, -10_deg, -90_deg)};

} // namespace VISION

namespace CLIMBER {
constexpr int LEFT_ID = 9;   // CHANGEME
constexpr int RIGHT_ID = 11; // CHANGEME
} // namespace CLIMBER
namespace CANDLE {
constexpr int CANDLE_ID = 10;
constexpr int NUM_LEDS = 39;
} // namespace CANDLE

namespace SHOOTER {
constexpr int LEFT_ID = 2;
constexpr int RIGHT_ID = 5;
constexpr int ANGLE_ID = 6;
constexpr int ANGLE2_ID = 2;
constexpr int CANCODER_ID = 13; // CHANGEME
constexpr std::pair<units::turn_t, units::turn_t> FENDER_RANGE = {0_tr, 1_tr};
constexpr double ANGLE_RATIO = 1; // CHANGEME
#ifdef BETABOT                    // Main robot config
constexpr units::turn_t FENDER_ANGLE = 11.5_tr;
constexpr units::turn_t AMP_ANGLE = 11_tr;
constexpr units::turns_per_second_t AMP_VELOCTITY = -6.5_tps;
constexpr units::turn_t REST_ANGLE = 0.242_tr;
constexpr units::turns_per_second_t SHOOTER_VELOCITY = 60_tps;
#endif
#ifndef BETABOT
constexpr units::turn_t REST_ANGLE = -0.5_tr;
constexpr units::turn_t FENDER_ANGLE = -11_tr;
constexpr units::turn_t AMP_ANGLE = -10_tr;
constexpr units::turns_per_second_t SHOOTER_VELOCITY = 80_tps;
#endif

constexpr int BELT_ID = 7;
constexpr units::turns_per_second_t LEFT_VELOCITY{10};  // CHANGEME;
constexpr units::turns_per_second_t RIGHT_VELOCITY{10}; // CHANGEME;

} // namespace SHOOTER

namespace DRIVE {
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

namespace CONFIG {
struct ModuleConfig {
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

#ifndef BETABOT
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
#endif // BETABOT

#ifdef BETABOT
#pragma message("Second Robot Config active")
/* -------------------------------------------------------------------------- */
/*                       BEGIN SECOND ROBOT CONFIGUATION                      */
/* -------------------------------------------------------------------------- */
constexpr ModuleConfig FL{60, 61, 14, -0.413_tr};
constexpr ModuleConfig FR{50, 51, 13, -0.182_tr }; // old value 0.32_tr - 0.5_tr
constexpr ModuleConfig BL{30, 31, 11,  0.317_tr};
constexpr ModuleConfig BR{40, 41, 12, -0.254_tr}; // old vaule -0.253_tr

/* -------------------------------------------------------------------------- */
/*                        END SECOND ROBOT CONFIGUATION                       */
/* -------------------------------------------------------------------------- */
#endif // BETABOT
} // namespace CONFIG
} // namespace DRIVE
} // namespace CONSTANTS
