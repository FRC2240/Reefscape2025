#include "swerve/Vision.h"
#include "swerve/Drivetrain.h"
#include <frc/DataLogManager.h>

Vision::Vision(std::function<units::degree_t()> get_angle_fn)
    : get_angle{get_angle_fn} {};

void Vision::log_metrics()
{
  // fps, cpu temp, ram usage, temp]
  std::vector<double> fps;
  std::vector<double> cpu_temp;
  std::vector<double> ram_usage;
  std::vector<double> temp;
  for (auto &i : m_limelight_vec)
  {
    // Get info here
    std::vector<double> hw = i.first->GetNumberArray("hw", std::vector<double>{0.0});
    if (hw[0] > 0.0)
    {
      if (hw[1] > 3.0)
      {
        overheat.Set(1);
      }
      else
      {
        overheat.Set(false);
      }
      fps.push_back(hw[0]);
      cpu_temp.push_back(hw[1]);
      ram_usage.push_back(hw[2]);
      temp.push_back(hw[3]);
    }
  }
  frc::SmartDashboard::PutNumberArray("vision/fps", fps);
  frc::SmartDashboard::PutNumberArray("vision/cpu_temp", cpu_temp);
  frc::SmartDashboard::PutNumberArray("vision/ram_usage", ram_usage);
  frc::SmartDashboard::PutNumberArray("vision/temp", temp);
}

std::vector<std::optional<frc::Pose2d>> Vision::get_bot_position()
{
  std::vector<std::optional<frc::Pose2d>> ret;
  for (auto &i : m_limelight_vec)
  {

    LimelightHelpers::SetRobotOrientation(i.second, get_angle().value(), 0, 0, 0, 0, 0);
    auto posevec = i.first->GetNumberArray("botpose_orb_wpiblue", std::vector<double>(6));
    frc::SmartDashboard::PutNumberArray("vision/" + i.second + "/pose", posevec);
    if (i.first->GetNumber("tv", 0.0) > 0.5 && posevec[0] > 0.01)
    {
      ret.push_back(frc::Pose2d{
          units::meter_t{posevec[0]},
          units::meter_t{posevec[1]},
          frc::Rotation2d{get_angle()}});
    }
  }

    std::vector<double> printvec_x;
  std::vector<double> printvec_y;
  for (auto &i : ret)
  {
    if (i)
    {
      printvec_x.push_back(i.value().X().value());
      printvec_y.push_back(i.value().Y().value());
      frc::SmartDashboard::PutNumberArray("pv/pos", std::vector<double>{i.value().X().value(), i.value().Y().value(), 0.0});
    }
  }
  // frc::SmartDashboard::PutNumberArray("pv/pos", std::vector<double>{0.0, 0.0, 0.0});
  frc::SmartDashboard::PutNumberArray("pv/x arr", printvec_x);
  frc::SmartDashboard::PutNumberArray("pv/y arr", printvec_y);
  return ret;
}

Vision::~Vision() = default;
