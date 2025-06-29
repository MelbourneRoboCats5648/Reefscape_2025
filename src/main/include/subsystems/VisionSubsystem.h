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

class VisionSubsystem : public frc2::SubsystemBase {
  public:
  VisionSubsystem(DriveSubsystem& driveSub);

  //Will be called periodically whenever the CommandScheduler runs.
  void Periodic() override;

  void AimAndRange();

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

    DriveSubsystem& m_drive;

  //FIX THESE CONSTANTS AND MOVE TO CONSTANTS.H AFTER (when ove change to inline)
  static constexpr double visionTurnKP = 0.5;
  static constexpr units::angle::degree_t reefDesiredAngle = 0.0_deg;
  static constexpr double visionStrafeKP = 1.0;
  static constexpr units::meter_t reefDesiredRange = 0.1_m;

};