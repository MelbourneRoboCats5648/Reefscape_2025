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


class VisionSubsystem : public frc2::SubsystemBase {
  public:
  VisionSubsystem(DriveSubsystem& driveSub);

  //Will be called periodically whenever the CommandScheduler runs.
  void Periodic() override;

  frc::Trajectory CreateTrajectory(frc::Pose2d targetPose);
  frc2::CommandPtr SwerveCommand(frc::Trajectory trajectory);

  frc2::CommandPtr Followtrajectory(frc::Trajectory trajectory);

  std::optional<frc::Pose2d> GetPoseAtTag(const int& reefTagID);

  //Will be called periodically whenever the CommandScheduler runs during simulation.
  void SimulationPeriodic() override;

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

    std::map<int, frc::Pose2d> poseMap = {
        {6, frc::Pose2d{ units::meter_t{530.49}, units::meter_t{130.17}, frc::Rotation2d{units::degree_t{300}} }},
        {7, frc::Pose2d{ units::meter_t{546.87}, units::meter_t {158.50}, frc::Rotation2d{units::degree_t{0}} }},
        {8, frc::Pose2d{ units::meter_t{530.49}, units::meter_t{186.83}, frc::Rotation2d{units::degree_t{60}} }},
        {9, frc::Pose2d{ units::meter_t{497.77}, units::meter_t{186.83}, frc::Rotation2d{units::degree_t{120}} }},
        {10, frc::Pose2d{ units::meter_t{481.39}, units::meter_t{158.50}, frc::Rotation2d{units::degree_t{180}} }},
        {11, frc::Pose2d{ units::meter_t{497.77}, units::meter_t{130.17}, frc::Rotation2d{units::degree_t{240}} }},
        {17, frc::Pose2d{ units::meter_t{160.39}, units::meter_t{130.17}, frc::Rotation2d{units::degree_t{240}} }},
        {18, frc::Pose2d{ units::meter_t{144.00}, units::meter_t{158.50}, frc::Rotation2d{units::degree_t{180}} }},
        {19, frc::Pose2d{ units::meter_t{160.39}, units::meter_t{186.83}, frc::Rotation2d{units::degree_t{120}} }},
        {20, frc::Pose2d{ units::meter_t{193.10}, units::meter_t{186.83}, frc::Rotation2d{units::degree_t{60}} }},
        {21, frc::Pose2d{ units::meter_t{209.49}, units::meter_t{158.50}, frc::Rotation2d{units::degree_t{0}} }},
        {22, frc::Pose2d{ units::meter_t{193.10}, units::meter_t{130.17}, frc::Rotation2d{units::degree_t{300}} }}
      };


    DriveSubsystem& m_drive;

  //FIX THESE CONSTANTS AND MOVE TO CONSTANTS.H AFTER (when ove change to inline)
  static constexpr double visionTurnKP = 0.05;
  static constexpr units::angle::degree_t reefDesiredAngle = 0.0_deg;
  static constexpr double visionStrafeKP = 0.2;
  static constexpr units::meter_t reefDesiredRange = 0.1_m;

};