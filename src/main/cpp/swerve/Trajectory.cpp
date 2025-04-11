#include "swerve/Trajectory.h"
#include "utility/MathUtils.h"
#include <vector>
#include <iostream>

#ifndef CFG_NO_DRIVEBASE
/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

// This is using lambdas in order to use setters at beginning of runtime & save
// performance later
/*static frc::HolonomicDriveController controller{
    frc2::PIDController{1, 0, 0},
    frc2::PIDController{1, 0, 0},*/
/*frc::ProfiledPIDController<units::radian>{
    //IMPORTANT: THIS DOES NOTHING.
    40, 0, 0,
    //0.8, 0.0, 0.0,
    frc::TrapezoidProfile<units::radian>::Constraints{

        CONSTANTS::DRIVE::TRAJ_MAX_ANGULAR_SPEED,
        CONSTANTS::DRIVE::TRAJ_MAX_ANGULAR_ACCELERATION}}};*/

frc::Timer m_trajTimer;

Trajectory::Trajectory(Drivetrain *drivetrain, Odometry *odometry,
                       frc2::CommandXboxController *stick, Vision *vision)
    : m_drivetrain{drivetrain}, m_odometry{odometry}, m_stick{stick},
      m_vision{vision}
{
  RobotConfig config = RobotConfig::fromGUISettings();

  AutoBuilder::configure(
      [this]() -> frc::Pose2d // Pose supplier
      {
        auto pose = m_odometry->getPose();
        frc::SmartDashboard::PutNumber("pp/X", pose.X().value());
        frc::SmartDashboard::PutNumber("pp/Y", pose.Y().value());

        return pose;
      },
      [this](frc::Pose2d pose) -> void // Pose resetter
      {
        frc::SmartDashboard::PutNumber("pp/rp/X", pose.X().value());
        frc::SmartDashboard::PutNumber("pp/rp/Y", pose.Y().value());
        m_odometry->resetPosition(
            pose, frc::Rotation2d(m_drivetrain->get_absolute_angle()));
      },
      [this]() -> frc::ChassisSpeeds // Chassis speeds supplier
      { return m_drivetrain->getRobotRelativeSpeeds(); },
      [this](frc::ChassisSpeeds speeds) -> void // Drive function
      {
        // Hey, bozos, we changed this on 2024-03-08 to make pathplanner work
        // Without this change pathplanner finds a barrier with remarkable
        // efficency due to not correcting right This is a result of inverting
        // the get_distance function
        return m_drivetrain->drive(-speeds);
      },
      std::make_shared<pathplanner::PPHolonomicDriveController>(
          pathplanner::PIDConstants(
              5.0, 0.0, 0.0),                      // Translation PID constants. Originally 1P
          pathplanner::PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                                                   // T: 1.75, 0, 0.0
                                                   // R: 0.625, 0.0, 0
          ),
      config, // The robot configuration
      []()
      {
        // Boolean supplier that controls when the path will be mirrored for the
        // red alliance This will flip the path being followed to the red side
        // of the field. THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        auto alliance = frc::DriverStation::GetAlliance();
        if (alliance)
        {
          return alliance.value() == frc::DriverStation::Alliance::kRed;
        }
        return false;
      },
      this);
}

frc2::CommandPtr Trajectory::manual_drive(bool field_relative)
{
  return frc2::cmd::Run(
      [this, &field_relative]
      {
        if (m_stick->Start().Get())
        {
          m_drivetrain->zero_yaw();
        }
        field_relative = true;
        units::meters_per_second_t left_right;
        units::meters_per_second_t front_back;
        units::angular_velocity::radians_per_second_t rot;

        if (m_stick->B().Get())
        {
          left_right = (frc::ApplyDeadband(m_stick->GetLeftX(), 0.1) *
                        CONSTANTS::DRIVE::SLOW_MODE) *
                       (CONSTANTS::DRIVE::TELEOP_MAX_SPEED);
          front_back = (frc::ApplyDeadband(m_stick->GetLeftY(), 0.1) *
                        CONSTANTS::DRIVE::SLOW_MODE) *
                       (CONSTANTS::DRIVE::TELEOP_MAX_SPEED);
          rot = (frc::ApplyDeadband(m_stick->GetRightX(), .1) *
                 CONSTANTS::DRIVE::SLOW_MODE) *
                (m_drivetrain->TELEOP_MAX_ANGULAR_SPEED);
        }
        else
        {
          left_right = frc::ApplyDeadband(m_stick->GetLeftX(), 0.1) *
                       (CONSTANTS::DRIVE::TELEOP_MAX_SPEED);
          front_back = frc::ApplyDeadband(m_stick->GetLeftY(), 0.1) *
                       (CONSTANTS::DRIVE::TELEOP_MAX_SPEED);
          rot = frc::ApplyDeadband(m_stick->GetRightX(), .1) *
                (m_drivetrain->TELEOP_MAX_ANGULAR_SPEED);
        }

        // Since heading 0 is always facing the red alliance wall and we flip
        // the gyro to compensate, invert the controls to compensate for the
        // compensation.
        if (frc::DriverStation::GetAlliance() &&
            frc::DriverStation::GetAlliance() ==
                frc::DriverStation::Alliance::kRed)
        {

          m_drivetrain->drive(-front_back, -left_right, rot, field_relative);
        }
        else
        {
          m_drivetrain->drive(front_back, left_right, rot, field_relative);
        }
        frc::SmartDashboard::PutBoolean("fldrel", field_relative);
      },
      {this});
}

frc2::CommandPtr Trajectory::extract(std::string auton)
{
  // fmt::println("{}", auton);
  return PathPlannerAuto(auton).ToPtr();
}

frc2::CommandPtr Trajectory::follow_live_path(frc::Pose2d goal_pose)
{
  return frc2::cmd::DeferredProxy([this, goal_pose]
                                  {

    frc::ChassisSpeeds speeds = m_odometry->getFieldRelativeSpeeds();


    std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses({frc::Pose2d(m_odometry->getPose().Translation(), units::math::atan(speeds.vy/speeds.vx)),
                                                                                                           goal_pose});

    // Makes a vector of waypoints. Waypoint 1 is the current pos, 2 is the goal
    std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses({m_odometry->getPose(),
                                                                           goal_pose});

    bool waypointsAreBad = false;
    auto numPoints = waypoints.size();

    for (auto i = 0; i < numPoints; i++) {
      // Check nextcontrol as long as its not the last point
      if (i != numPoints - 1 && !waypoints[i].nextControl.has_value()) {
        waypointsAreBad = true;
        break;
      }
      
      // Check prevcontrol if it's not the first point
      if (i != 0 && !waypoints[i].prevControl.has_value()) {
        waypointsAreBad = true;
        break;
      }
    }

    if (waypointsAreBad) {
      std::cout << "Bad waypoints" << std::endl;
      return frc2::cmd::None();
    }

    // The constraints for the path. TODO CHANGE IT
    PathConstraints constraints(1_mps, 1_mps_sq, 180_deg_per_s, 270_deg_per_s_sq);

    frc::ChassisSpeeds speeds = m_odometry->getFieldRelativeSpeeds();
    units::meters_per_second_t vel = static_cast<units::meters_per_second_t>(MathUtils::pythag(speeds.vx(), speeds.vy()));

    auto path = std::make_shared<PathPlannerPath>(
        waypoints,
        constraints,
        IdealStartingState(vel, units::math::atan((speeds.vy/speeds.vx))), // The ideal starting state, might need to be changed
        GoalEndState(0.0_mps, goal_pose.Rotation())                // Goal end state. You can set a holonomic rotation here.
    );

    path->preventFlipping = true;

    return AutoBuilder::followPath(path).AlongWith(frc2::cmd::RunOnce([] {std::cout << "Follow called" << std::endl;}));
});
}

frc2::CommandPtr Trajectory::reef_align_command(CONSTANTS::FIELD_POSITIONS::REEF_SIDE_SIDE side_side)
{
  return frc2::cmd::DeferredProxy([this, side_side]
                                  {
  try
  {

    std::cout << "Align command called" << std::endl;

    int nearest_face = m_odometry->get_nearest_reef_side();
    frc::Pose2d botPos = m_odometry->getPose();

    // If the distance to the nearest reef face is too large, do not continue.
    if (MathUtils::getDistance(m_odometry->get_reef_face_pos(nearest_face), botPos) > CONSTANTS::FIELD_POSITIONS::EFFECTIVE_DISTANCE)
    {
      std::cout << "Too far from reef" << std::endl;
      return frc2::cmd::RunOnce([] {}); // Empty command to exit early.
    }

    const frc::Pose2d goal_pose = m_odometry->get_alignment_position(nearest_face, side_side);

    m_timer.Restart();

    return follow_live_path(goal_pose).Repeatedly().Until([this, goal_pose] -> bool
                                                          { return MathUtils::getDistance(goal_pose, m_odometry->getPose()) < CONSTANTS::FIELD_POSITIONS::PATH_FINISHED_DIST_THRESHOLD &&
                                                                   units::math::abs(goal_pose.Rotation().Degrees() - m_odometry->getPose().Rotation().Degrees()) < CONSTANTS::FIELD_POSITIONS::PATH_FINISHED_ANGLE_THRESHOLD; })
        .Until([this] -> bool
               {
                                      const double t = CONSTANTS::FIELD_POSITIONS::DRIVER_OVERRIDE_THRESHOLD;
                                      return m_timer.HasElapsed(0.5_s) && (std::abs(m_stick->GetRightX()) > t || 
                                             std::abs(m_stick->GetRightY()) > t || 
                                             std::abs(m_stick->GetLeftX()) > t || 
                                             std::abs(m_stick->GetLeftY()) > t); })
        .AndThen([this]
                 { m_timer.Stop(); });
  }
  catch (const std::exception &e)
  {
    std::cout << "Exception in reef_align_command: " << e.what() << std::endl;
    return frc2::cmd::RunOnce([] {});
  }
  catch (const char *e)
  {
    std::cout << "char Exception in reef_align_command: " << e << std::endl;
    return frc2::cmd::RunOnce([] {});
  }
  catch (const std::string &e)
  {
    std::cout << "Exception in reef_align_command: " << e << std::endl;
    return frc2::cmd::RunOnce([] {});
  }
  catch (const int &e)
  {
    std::cout << "Exception in reef_align_command: " << e << std::endl;
    return frc2::cmd::RunOnce([] {});
  }
  catch (std::runtime_error &e){
    std::cout << "Exception in reef_align_command: " << e.what() << std::endl;
  }
  catch (...)
  {
    
    std::cout << "unkown exception" << std::endl;
  } });
}

#endif
