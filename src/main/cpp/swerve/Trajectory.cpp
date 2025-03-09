#include "swerve/Trajectory.h"
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
              5.0, 0.0, 0.0),                       // Translation PID constants. Originally 1P
          pathplanner::PIDConstants(5.0, 0.0, 0) // Rotation PID constants
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
        if (m_stick->GetLeftTriggerAxis() > 0.5)
        {
          field_relative = false;
          //fmt::println("here");
        }
        else
        {
          field_relative = true;
        }
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

#endif
