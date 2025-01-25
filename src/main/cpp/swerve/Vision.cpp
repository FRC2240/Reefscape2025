#include "swerve/Vision.h"
#include "swerve/Drivetrain.h"
#include <frc/DataLogManager.h>

Vision::Vision(std::function<units::degree_t()> get_angle_fn)
    : get_angle{get_angle_fn} {};

std::vector<std::optional<frc::Pose2d>> Vision::get_bot_position()
{
  auto aft_results = m_aft_limelight->GetNumberArray("botpose_wpiblue",
                                                     std::vector<double>(6));

  frc::SmartDashboard::PutNumber("vision step", 1);
  std::vector<std::optional<frc::Pose2d>> ret;
  for (auto &i : m_photoncam_vec)
  {
    frc::SmartDashboard::PutNumber("vision step", 2);
    auto result = i.camera->GetLatestResult();
    frc::SmartDashboard::PutNumber("vision step", 2.5);
    frc::SmartDashboard::PutNumber("vision is present", result.HasTargets());
    if (result.MultiTagResult())
    {

      frc::SmartDashboard::PutNumber("vision step", 3);
      auto pose = i.multitag_estimator.Update(result);
      if (pose)
      {

        frc::SmartDashboard::PutNumber("vision step", 4);
        frc::SmartDashboard::PutNumber("pv/x", pose.value().estimatedPose.X().value());
        frc::SmartDashboard::PutNumber("pv/y", pose.value().estimatedPose.Y().value());
        frc::SmartDashboard::PutNumber("pv/get_angle()", get_angle().value());
        // if (CONSTANTS::IN_THRESHOLD<units::degree_t>(
        //         pose.value().estimatedPose.Rotation().ToRotation2d().Degrees(),
        //         get_angle(), 3_deg))
        // {
        ret.push_back(pose.value().estimatedPose.ToPose2d());
        // }
      }
    }
    else if (result.HasTargets())
    {
      auto pose = i.singletag_estimator.Update(result);
      ret.push_back(pose.value().estimatedPose.ToPose2d());
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

std::optional<units::degree_t> Vision::get_neural_net_angle()
{
  std::string_view tclass = "tclass: " + m_fore_limelight->GetString("tclass", "NULL");
  frc::DataLogManager::Log(tclass);
  if (m_fore_limelight->GetString("tclass", "ERROR") ==
      "note") // Assuming "note" is the correct key
  {
    // fmt::println("here!!!");
    units::degree_t tx{m_fore_limelight->GetNumber("tx", 0.0)};
    // Target is valid, return info

    return std::optional<units::degree_t>{tx};
  }
  else
  {
    // fmt::println("there!!!");
    return {};
  }
}

Vision::~Vision() = default;
