#pragma once

#include "swerve/Drivetrain.h"
#include <frc/DataLogManager.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <pathplanner/lib/path/PathPoint.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include "Constants.h"
#include <functional>
#include <cmath>
#include "swerve/Odometry.h"
#include "swerve/Vision.h"
#include <frc/DriverStation.h>

#include <frc/controller/HolonomicDriveController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/SubsystemBase.h>
#include <chrono>
#include <thread>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <pathplanner/lib/config/PIDConstants.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include <frc2/command/button/CommandXboxController.h>
#include "frc2/command/DeferredCommand.h"

#ifndef CFG_NO_DRIVEBASE
using namespace pathplanner; // PathPlanner keeps everything hidden behind 2 sets of namespaces so it's safe to remove the first layer

class Trajectory : public frc2::SubsystemBase
{
public:
    Trajectory(
        Drivetrain *drivetrain,
        Odometry *odometry,
        frc2::CommandXboxController *stick,
        Vision *vision);

   

    // Note: a 2023 comment means it is Moonwalker Specific and can be safely removed.

    const PathConstraints DEFAULT_CONSTRAINTS = PathConstraints(14_fps, 7_fps_sq, 360_deg_per_s, 720_deg_per_s_sq);
    // constexpr PathConstraints DEFAULT_CONSTRAINTS = PathConstraints(CONSTANTS:)

    frc2::CommandPtr manual_drive(bool field_relative = true);

    // Autoalign stuff
    frc2::CommandPtr follow_live_path(frc::Pose2d goal_pose);
    frc2::CommandPtr reef_align_command(CONSTANTS::FIELD_POSITIONS::REEF_SIDE_SIDE side_side);


    frc2::CommandPtr extract(std::string auton);

private:
int attempts = 0;
    int cyclecounter = 0;
    Drivetrain *m_drivetrain;
    Odometry *m_odometry;
    frc2::CommandXboxController *m_stick;
    Vision *m_vision;
    frc::Timer m_timer;

    units::degree_t desired_angle;
};
#endif
