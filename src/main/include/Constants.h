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
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <vector>

// #define COLFAX_BOT

// #define SABERTOOTH
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

    double GetP() { return kP; }
    void SetP(double val) { kP = val; }

    double GetI() { return kI; }
    void SetI(double val) { kI = val; }

    double GetD() { return kD; }
    void SetD(double val) { kD = val; }

    double GetS() { return kS; }
    void SetS(double val) { kS = val; }

    void SetG(double val) { kG = val; }
    double GetG() { return kG; }

    void SetMin(double val) { min = val; }
    double GetMin() { return min; }

    void SetMax(double val) { max = val; }
    double GetMax() { return max; }
  };

  namespace CORAL
  {

    static const std::string LIMELIGHT_ID = "limelight-aft";
    constexpr units::meters_per_second_t APPROACH_SPEED = 0.25_mps;

  } // namespace CORAL

  namespace MANIPULATOR_STATES
  {
    constexpr units::turn_t POST_SCORE_DELTA = 5.4_tr - 18.4_tr;
    // Represents the arm and wrist position required to score somewhere
    struct ManipulatorState
    {
      units::turn_t elevtor_pos;
      units::turn_t wrist_pos;

      // I don't know why this needs to be overriden but it does.  ¯\_(-_-)_/¯
      bool operator==(ManipulatorState other)
      {
        return (IN_THRESHOLD<units::turn_t>(this->elevtor_pos, other.elevtor_pos,
                                            0.00005_tr) &&
                IN_THRESHOLD<units::turn_t>(this->wrist_pos, other.wrist_pos,
                                            0.00005_tr));
      }
    };
    constexpr ManipulatorState L1{0_tr, 0_tr};
    constexpr ManipulatorState L2{12.61_tr, 5.38_tr}; // 18.4 follow-through
    constexpr ManipulatorState L3{25.75_tr, 4.88_tr};
    constexpr ManipulatorState L4{46.75_tr, 4.58_tr};
    constexpr ManipulatorState IDLE{13.68_tr, 19.7_tr};
    constexpr ManipulatorState IDLE_W_GP{12.329_tr, 19.7_tr};
    constexpr ManipulatorState INTAKE{7.79_tr, 20.3_tr};
    constexpr ManipulatorState POST_SCORE{0_tr, 11.3_tr};
    constexpr ManipulatorState ALGAE_L2{0_tr, 0_tr};
    constexpr ManipulatorState ALGAE_L3{0_tr, 0_tr};
    constexpr ManipulatorState PROCESSOR{0_tr, 0_tr};
    constexpr ManipulatorState NET{0_tr, 0_tr};

  } // namespace MANIPULATOR_STATES

  namespace GRABBER
  {
    constexpr int MOTOR_ID = 20; // changeme
    // Configuration page: http://10.22.40.2:5812
    constexpr int TOF_ID = 32; // changeme

    static CONSTANTS::PidCoeff PID = {1, 0, 0}; // values are TBD

    // intake
    constexpr units::turns_per_second_t INTAKE_ALGAE_VELOCITY = 10_tps; // TBD
    constexpr units::turns_per_second_t INTAKE_CORAL_VELOCITY = 10_tps; // TBD
    constexpr units::millimeter_t DEFAULT_DIST_TOF = 20_mm;             // TBD

    // extake
    constexpr units::turns_per_second_t EXTAKE_VELOCITY = 10_tps; // TBD
    constexpr units::second_t EXTAKE_TIME = 1_s;                  // TBD
  } // namespace GRABBER

  namespace ELEVATOR
  {
    constexpr units::angle::turn_t POSITION_THRESHOLD = 2_tr;
    constexpr units::angle::turn_t BOTTOM_POS = 0_tr;

    constexpr units::angle::turn_t TOP_POS = 46.75_tr;
    constexpr int LEFT_ID = 20;
    constexpr int RIGHT_ID = 21;
    constexpr double DEADBAND_THRESHOLD = 0.1;
    static const PidCoeff PidValue = {
        40,
        0,
        10,
        4.5,
        0,
    };

    constexpr units::angle::turn_t OFFSET_AMOUNT = 3_tr;

    constexpr units::angular_velocity::turns_per_second_t JOYSTICK_SPEED = 1_tps;
    namespace PRESETS
    {
      constexpr units::angle::turn_t BOTTOM = BOTTOM_POS;
      constexpr units::angle::turn_t TOP = TOP_POS;
    } // namespace PRESETS
  } // namespace ELEVATOR

  namespace WRIST
  {
    constexpr int WRIST_ID = 5;
    constexpr units::angle::turn_t DEFAULT_POSITION = 0_tr;

    constexpr units::current::ampere_t PEAK_TORQUE_CURRENT = 12_A;
    constexpr units::current::ampere_t PEAK_STATOR_CURRENT = 20_A;
    constexpr units::angular_velocity::turns_per_second_t PEAK_VELOCITY = 10_tps;
    // This is the default PID values for the wrist motor
    static const PidCoeff PidValue = {8, 0, 1};

    constexpr units::angle::degree_t OFFSET_AMOUNT = 1_tr;

    constexpr units::angle::degree_t POSITION_THRESHOLD = 5_deg;
  }; // namespace WRIST

  namespace VISION
  {
    /* Transforms from 2024:
    static const auto LEFT_CAMERA_A_TF = frc::Transform3d{
      0.151_m, 0.319_m, 0.578_m, frc::Rotation3d(180_deg, -10_deg, 90_deg)};
    static const auto RIGHT_CAMERA_A_TF = frc::Transform3d{
      0.151_m, -0.319_m, 0.578_m, frc::Rotation3d(180_deg, -10_deg, -90_deg)};
    */

    static const auto LEFT_CAMERA_A_TF = frc::Transform3d{
        -1.25_in, 10.75_in, 39.488_in, frc::Rotation3d(0_deg, 0_deg, 10_deg)};
    static const auto RIGHT_CAMERA_A_TF = frc::Transform3d{
        -1.25_in, -10.75_in, 39.488_m, frc::Rotation3d(0_deg, 0_deg, -10_deg)};

  } // namespace VISION

  namespace CLIMBER
  {
    constexpr int CLIMBER_ID = 7;

    constexpr units::angle::turn_t POSITION_THRESHOLD = 2_tr;
    constexpr units::angle::turn_t DEFAULT_POS = 0_tr;
    constexpr units::angle::turn_t EXTEND_POS = 30_tr;
    constexpr units::angle::turn_t CLIMB_POS = 15_tr;

    static const PidCoeff PidValue = {4, 0, 0};

  } // namespace CLIMBER

  namespace CANDLE
  {
    constexpr int CANDLE_ID = 10;
    constexpr int NUM_LEDS = 39;
  } // namespace CANDLE

  namespace DRIVE
  {
    constexpr units::second_t BRAKE_TIME = 10_s;
    constexpr units::meters_per_second_t ROBOT_MAX_SPEED = 14.533_fps;
    constexpr units::radians_per_second_t ROBOT_MAX_ANGULAR_SPEED{std::numbers::pi *
                                                                  3};
    constexpr units::meters_per_second_t TELEOP_MAX_SPEED = ROBOT_MAX_SPEED;
    constexpr units::radians_per_second_t TELEOP_MAX_ANGULAR_SPEED{
        std::numbers::pi * 3};
    constexpr units::meters_per_second_t TRAJ_MAX_SPEED = ROBOT_MAX_SPEED;
    constexpr units::acceleration::meters_per_second_squared_t
        TRAJ_MAX_ACCELERATION = TRAJ_MAX_SPEED / 0.5_s;
    constexpr units::radians_per_second_t TRAJ_MAX_ANGULAR_SPEED =
        CONSTANTS::DRIVE::ROBOT_MAX_ANGULAR_SPEED;
    constexpr units::radians_per_second_squared_t TRAJ_MAX_ANGULAR_ACCELERATION{
        std::numbers::pi};
    static constexpr auto WHEEL_CIRCUMFERENCE = 12.02_in / 1.0_tr;
    // static constexpr auto WHEEL_CIRCUMFERENCE = 11.992_in / 1.0_tr;
    constexpr int GYRO_ID = 48; // CHANGEME
    constexpr double SLOW_MODE = 0.5;
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
      constexpr ModuleConfig FL{60, 61, 14, 0.306867_tr};
      constexpr ModuleConfig FR{50, 51, 13, 0.1819_tr}; // old value 0.32_tr - 0.5_tr
      constexpr ModuleConfig BL{30, 31, 11, 0.18872_tr};
      constexpr ModuleConfig BR{40, 41, 12, 0.42531_tr}; // old vaule -0.253_tr

/* -------------------------------------------------------------------------- */
/*                        END FIRST ROBOT CONFIGURATION                       */
/* -------------------------------------------------------------------------- */
#endif // NOT SABERTOOTH

#ifdef SABERTOOTH
#pragma message("Second Robot Config active")
      /* -------------------------------------------------------------------------- */
      /*                       BEGIN SECOND ROBOT CONFIGUATION                      */
      /* -------------------------------------------------------------------------- */
      constexpr ModuleConfig FL{60, 61, 14, -0.413_tr};
      constexpr ModuleConfig FR{50, 51, 13, 0.19_tr}; // old value 0.32_tr - 0.5_tr
      constexpr ModuleConfig BL{30, 31, 11, 0.317_tr};
      constexpr ModuleConfig BR{40, 41, 12, -0.236_tr}; // old vaule -0.253_tr

/* -------------------------------------------------------------------------- */
/*                        END SECOND ROBOT CONFIGUATION                       */
/* -------------------------------------------------------------------------- */
#endif // SABERTOOTH
    } // namespace CONFIG
  } // namespace DRIVE
} // namespace CONSTANTS
