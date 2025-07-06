#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <Constants.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/PhotonCamera.h>

#include <frc/filter/SlewRateLimiter.h>

#include "subsystems/DriveSubsystem.h"

#include <photon/PhotonUtils.h>
#include <set>
#include <map>
#include "frc/geometry/Rotation2d.h"


#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <frc2/command/SwerveControllerCommand.h>


#include <frc/geometry/Pose2d.h>
#include <networktables/StructTopic.h>

class VisionSubsystem : public frc2::SubsystemBase {
  public:
  VisionSubsystem(DriveSubsystem& driveSub);

  //Will be called periodically whenever the CommandScheduler runs.
  void Periodic() override;

  frc::Trajectory CreateTrajectory(frc::Pose2d targetPose);

  frc2::CommandPtr Followtrajectory(frc::Trajectory trajectory);

  std::optional<frc::Pose2d> GetPoseAtTag(const int& reefTagID);
  
  frc2::CommandPtr MoveToTarget(ReefPosition position);

  //Will be called periodically whenever the CommandScheduler runs during simulation.
  void SimulationPeriodic() override;

  frc::Pose2d GetLeftPose(const frc::Pose2d& tagPose); // get robot's pose to the left of a tag given its outward-facing pose
  frc::Pose2d GetRightPose(const frc::Pose2d& tagPose); // get robot's pose to the right of a tag given its outward-facing pose

  private:
    photon::PhotonCamera camera{"photonvision"};
    photon::PhotonPoseEstimator photonEstimator
    {
      VisionConstants::kTagLayout,
      photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
      VisionConstants::kRobotToCam //FIX ME CHECK THE ROBOT TO CAM MEASUREMENTS
    };
    photon::PhotonPipelineResult m_latestResult;

    // Declare and initialize a map

    std::map<int, frc::Pose2d> poseMap = { //TODO need to remove tag id 1 and 2
      {1, frc::Pose2d{ units::centimeter_t{657.37}, units::centimeter_t{25.80}, frc::Rotation2d{units::degree_t{126}} }},
      {2, frc::Pose2d{ units::centimeter_t{657.37}, units::centimeter_t{291.20}, frc::Rotation2d{units::degree_t{234}} }},

        {6, frc::Pose2d{ units::centimeter_t{530.49}, units::centimeter_t{130.17}, frc::Rotation2d{units::degree_t{300}} }},
        {7, frc::Pose2d{ units::centimeter_t{546.87}, units::centimeter_t {158.50}, frc::Rotation2d{units::degree_t{0}} }},
        {8, frc::Pose2d{ units::centimeter_t{530.49}, units::centimeter_t{186.83}, frc::Rotation2d{units::degree_t{60}} }},
        {9, frc::Pose2d{ units::centimeter_t{497.77}, units::centimeter_t{186.83}, frc::Rotation2d{units::degree_t{120}} }},
        {10, frc::Pose2d{ units::centimeter_t{481.39}, units::centimeter_t{158.50}, frc::Rotation2d{units::degree_t{180}} }},
        {11, frc::Pose2d{ units::centimeter_t{497.77}, units::centimeter_t{130.17}, frc::Rotation2d{units::degree_t{240}} }},
        {17, frc::Pose2d{ units::centimeter_t{160.39}, units::centimeter_t{130.17}, frc::Rotation2d{units::degree_t{240}} }},
        {18, frc::Pose2d{ units::centimeter_t{144.00}, units::centimeter_t{158.50}, frc::Rotation2d{units::degree_t{180}} }},
        {19, frc::Pose2d{ units::centimeter_t{160.39}, units::centimeter_t{186.83}, frc::Rotation2d{units::degree_t{120}} }},
        {20, frc::Pose2d{ units::centimeter_t{193.10}, units::centimeter_t{186.83}, frc::Rotation2d{units::degree_t{60}} }},
        {21, frc::Pose2d{ units::centimeter_t{209.49}, units::centimeter_t{158.50}, frc::Rotation2d{units::degree_t{0}} }},
        {22, frc::Pose2d{ units::centimeter_t{193.10}, units::centimeter_t{130.17}, frc::Rotation2d{units::degree_t{300}} }}
      };


    DriveSubsystem& m_drive;

  //FIX THESE CONSTANTS AND MOVE TO CONSTANTS.H AFTER (when ove change to inline)
  static constexpr double visionTurnKP = 0.05;
  static constexpr units::angle::degree_t reefDesiredAngle = 0.0_deg;
  static constexpr double visionStrafeKP = 0.2;
  static constexpr units::meter_t reefDesiredRange = 0.1_m;

  nt::StructPublisher<frc::Pose2d> m_posePublisher;

  Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose);

};