#pragma once

#ifndef CFG_NO_DRIVEBASE

#include "swerve/SwerveModule.h"
#include "swerve/ngr.h"
#include "utility/MotorUtils.h"

#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/BuiltInAccelerometer.h>
#include <frc/SPI.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <wpi/array.h>

#include <fmt/format.h>
#include <iostream>

class Drivetrain
{
public:
    frc::SwerveDriveKinematics<4> kinematics{
        frc::Translation2d{12.18_in, 12.18_in},
        frc::Translation2d{12.18_in, -12.18_in},
        frc::Translation2d{-12.18_in, 12.18_in},
        frc::Translation2d{-12.18_in, -12.18_in}};

    frc::BuiltInAccelerometer acc;
    Drivetrain();

    /// @brief  outputs acceleration to NT
    void log_accel();

    /// @brief Flips the perspective of the drivetrain, so it rotates it's pose by
    /// 180 degrees.
    void flip();
    /// @brief Resets the navx so it's current orentation becomes 0 degrees
    void zero_yaw();

    double get_pitch();

    void print_angle();

    void debug_angles();
    /// @brief Snaps to facing away from the driverstation
    /// @return True if the robot is facing away from the driverstation. False if
    /// not.
    bool human_player_snap();

    /******************************************************************/
    /*                        Public Constants                        */
    /******************************************************************/

    // Absolute max module speed
    static constexpr units::meters_per_second_t MODULE_MAX_SPEED = 13.9_fps;

    /*
    Max effective speed considering pi radians/second max angular speed
    ROBOT SAT: 9.535f/s
    */

    // Formula for determing ROBOT_MAX_SPEED is Wheel Max Speed = Robot Max Speed
    // + Omega max speed * distance of module from center Or Robot Max Speed = Max
    // wheel speed - Omega max speed * distance from center Distance of module
    // from center is 1.294ft

    // Max effective linear speed
    static constexpr units::meters_per_second_t ROBOT_MAX_SPEED = 14.533_fps;
    static constexpr units::radians_per_second_t ROBOT_MAX_ANGULAR_SPEED{
        std::numbers::pi * 2.25};

    static constexpr units::meters_per_second_t TELEOP_MAX_SPEED =
        ROBOT_MAX_SPEED;
    static constexpr units::radians_per_second_t TELEOP_MAX_ANGULAR_SPEED{
        std::numbers::pi * 1};
    static constexpr units::meters_per_second_t TRAJ_MAX_SPEED = ROBOT_MAX_SPEED;
    static constexpr units::acceleration::meters_per_second_squared_t
        TRAJ_MAX_ACCELERATION = TRAJ_MAX_SPEED / 0.5_s;
    static constexpr units::radians_per_second_t TRAJ_MAX_ANGULAR_SPEED =
        ROBOT_MAX_ANGULAR_SPEED;
    static constexpr units::radians_per_second_squared_t
        TRAJ_MAX_ANGULAR_ACCELERATION{std::numbers::pi};

    static constexpr auto ROTATE_P =
        2; // Modifier for rotational speed -> (degree * ROTATE_P) / 1sec

    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/
    void init();
    void zero_adjustment();
    double get_offset();

    // Returns values with 0 being front and positive angles going CW
    [[nodiscard]] units::degree_t getAngle();

    [[nodiscard]] frc::Rotation2d getCCWHeading();

    [[nodiscard]] frc::Rotation2d getCWHeading();

    [[nodiscard]] wpi::array<double, 4> getDriverTemps();

    [[nodiscard]] wpi::array<double, 4> getTurnerTemps();

    [[nodiscard]] frc::ChassisSpeeds getRobotRelativeSpeeds();

    [[nodiscard]] wpi::array<frc::SwerveModuleState, 4> getModuleStates();

    [[nodiscard]] wpi::array<frc::SwerveModulePosition, 4> getModulePositions();

    // Handles inversing
    bool snap_to_zero();
    units::degree_t get_absolute_angle();

    void tankDrive(double const &x_speed, double const &y_speed);

    /// @brief The entrypoint into driving in a normal swerve fasion.
    /// Should be "closest to the user".
    /// Calls the ChassisSpeeds method
    /// @param xSpeed Desired horizontal speed (side to side)
    /// @param ySpeed Desired "vertical" speed (front to back)
    /// @param rot How fast it should rotate to the right
    /// @param fieldRelative Driving mode, true being field centric.
    void drive(units::meters_per_second_t const &xSpeed,
               units::meters_per_second_t const &ySpeed,
               units::radians_per_second_t const &rot, bool const &fieldRelative);

    /// @brief Takes the desired Chassis speeds, and figures out how each module
    /// should move.
    /// @param speeds The desired speeds the robot will move in
    void drive(frc::ChassisSpeeds const &speeds);

    /// @brief The final step in the drive stack.
    /// Moves each module to the required speed, given by the last step.
    /// @param states How fast should each module be moving and where.
    void drive(wpi::array<frc::SwerveModuleState, 4> states);

    void stop();

    // For theta, positive is CCW
    void faceDirection(units::meters_per_second_t const &dx,
                       units::meters_per_second_t const &dy,
                       units::degree_t const &theta, 
                       bool const &field_relative,
                       double const &rot_p = ROTATE_P,
                       units::degrees_per_second_t const &max_rot_speed =
                           TELEOP_MAX_ANGULAR_SPEED);

    void faceClosest(units::meters_per_second_t const &dx,
                     units::meters_per_second_t const &dy,
                     bool const &field_relative, double const &rot_p = ROTATE_P,
                     units::degrees_per_second_t const &max_rot_speed =
                         TELEOP_MAX_ANGULAR_SPEED);

    void tuneTurner(units::degree_t const &desired_angle);

    void manualPercentOutput(double const &percent_output);

    void manualVelocity(double const &velocity_ticks_per_100ms);

    bool face_direction(units::degree_t tgt, double feedback_device);

    bool face_direction(units::degree_t tgt, units::meters_per_second_t dx=0_mps, units::meters_per_second_t dy=0_mps);
    // bool face_direction(units::degree_t tgt);
    ctre::phoenix6::hardware::Pigeon2 gyro{CONSTANTS::DRIVE::GYRO_ID, "swervecan"};

private:
    // ctre::phoenix6::hardware::Pigeon2 gyro{CONSTANTS::DRIVE::GYRO_ID, "rio"};
    //  AHRS navx{frc::SPI::Port::kMXP};
    CONSTANTS::PidCoeff pid_coef{2.75, 0.0, 0.6, 0.0, 0.0, -1, 1};
    frc::PIDController turn_pid{pid_coef.kP, pid_coef.kI, pid_coef.kD};
    CONSTANTS::PidCoeff pid_coral_coef{6.0, 0.0, 0.0, 0.0, 0.0, -1, 1};
    frc::PIDController turn_coral_pid{pid_coral_coef.kP, pid_coral_coef.kI,
                                      pid_coral_coef.kD};
};

#endif
