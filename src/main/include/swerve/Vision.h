#pragma once
#include "Constants.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <frc/DriverStation.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <stdlib.h>
#include <units/angle.h>
#include <frc/Alert.h>
class Vision
{
public:
    /*
    At the time of writing, Vision must be capable of:
    1. Translation accuracy within 6in
    1.a. Doing so no slower than 0.9s
    2. determining the angle of a note to within 3 degrees
    3. Determing the distance to a note within 29 inches
    4. Determing the angle to an apriltag within 2 degrees
    */

    Vision(std::function<units::degree_t()> get_angle_fn);
    ~Vision();

    // Optionals are used liberally in this file due to the uncertain nature of
    // vison. See
    // https://stackoverflow.com/questions/16860960/how-should-one-use-stdoptional
    // for more info

    // Returns a vector of optionals of camera outputs.
    // The caller is expected to handle absent data, not the function.
    std::vector<std::optional<frc::Pose2d>> get_bot_position();

    // Returns the angle to a gp
    std::optional<units::degree_t> get_coral_angle();

    // Returns angle to apriltag 4 or 7, depending on alliance color.
    // Could be modified to work with 3 and 8 as well
    std::optional<units::degree_t> get_apriltag_angle();

    void log_metrics();

private:
    struct PhotonGroup
    {
        std::shared_ptr<photon::PhotonCamera> camera;
        photon::PhotonPoseEstimator multitag_estimator;
        photon::PhotonPoseEstimator singletag_estimator;
        PhotonGroup(std::shared_ptr<photon::PhotonCamera> acamera,
                    photon::PhotonPoseEstimator amultitag_estimator,
                    photon::PhotonPoseEstimator asingletag_estimator)
            : camera{acamera},
              multitag_estimator{amultitag_estimator},
              singletag_estimator{asingletag_estimator}
        {
        }
    };

    std::function<units::degree_t()> get_angle;
    bool is_hardware_zoomed = 0;

    std::shared_ptr<photon::PhotonCamera> m_left_camera_a =
        std::make_shared<photon::PhotonCamera>("left_camera_a");

    std::shared_ptr<photon::PhotonCamera> m_right_camera_a =
        std::make_shared<photon::PhotonCamera>("right_camera_a");

    frc::AprilTagFieldLayout layout =
        frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025Reefscape);

    photon::PhotonPoseEstimator m_left_estimator_a{
        layout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
        CONSTANTS::VISION::LEFT_CAMERA_A_TF};

    photon::PhotonPoseEstimator m_single_left_estimator{
        layout, photon::PoseStrategy::LOWEST_AMBIGUITY,
        CONSTANTS::VISION::LEFT_CAMERA_A_TF};

    photon::PhotonPoseEstimator m_right_estimator_a{
        layout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
        CONSTANTS::VISION::RIGHT_CAMERA_A_TF};

    photon::PhotonPoseEstimator m_single_right_estimator{
        layout, photon::PoseStrategy::LOWEST_AMBIGUITY,
        CONSTANTS::VISION::RIGHT_CAMERA_A_TF};

    std::vector<PhotonGroup>
        m_photoncam_vec = {{m_left_camera_a, m_left_estimator_a, m_single_left_estimator},
                           {m_right_camera_a, m_right_estimator_a, m_single_right_estimator}};

    std::shared_ptr<nt::NetworkTable> m_aft_limelight =
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-aft");

    std::shared_ptr<nt::NetworkTable> m_fore_limelight =
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-fore");

    std::vector<std::shared_ptr<nt::NetworkTable>> m_limelight_vec = {m_aft_limelight};

    frc::Alert overheat{"Limelight overheating", frc::Alert::AlertType::kWarning};
};
