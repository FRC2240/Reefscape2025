#pragma once

#include "swerve/SwerveModule.h"
#include "swerve/Drivetrain.h"
#include "swerve/ngr.h"
#include "Constants.h"
#include <iostream>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
// #include <TimeOfFlight.h>
#include <frc/DriverStation.h>
#include "swerve/Vision.h"
#include <frc2/command/Commands.h>

#ifndef CFG_NO_DRIVEBASE
class Odometry
{
private:
    Drivetrain *m_drivetrain;
    // std::shared_ptr<nt::NetworkTable> m_limelight =
    Vision *m_vision;
    // nt::NetworkTableInstance::GetDefault().GetTable("limelight-dev");?
    frc::Field2d field2d;

public:
    // static frc::SwerveDriveKinematics<4> *kinematics_ptr;
    Odometry(Drivetrain *drivetrain, Vision *vision);
    void putField2d();

    frc2::CommandPtr set_pose_cmd(frc::Pose2d pose);

    std::shared_ptr<nt::NetworkTable> m_limelight = nt::NetworkTableInstance::GetDefault().GetTable("limelight-dev");
    frc::SwerveDrivePoseEstimator<4> estimator{
        m_drivetrain->kinematics,
        frc::Rotation2d(m_drivetrain->getAngle()),
        wpi::array<frc::SwerveModulePosition, 4>{
            frc::SwerveModulePosition{},
            frc::SwerveModulePosition{},
            frc::SwerveModulePosition{},
            frc::SwerveModulePosition{}},
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_rad))};

    [[nodiscard]] frc::Pose2d getPose();

    void reset_position_from_vision(const frc::Pose2d &bot_pose);

    void reset_from_distance();

    void update();

    void resetPosition(const frc::Pose2d &pose,
                       const frc::Rotation2d &gyroAngle);

    [[nodiscard]] frc::FieldObject2d *getField2dObject(std::string_view name);

    [[nodiscard]] frc::ChassisSpeeds const getFieldRelativeSpeeds();

    void add_vision_measurment(const frc::Pose2d &pose);

    void update_from_vision();

    std::optional<units::degree_t> get_coral();

    std::optional<units::meter_t> get_dist_to_tgt();

    units::turn_t get_shooter_angle();
};
#endif
