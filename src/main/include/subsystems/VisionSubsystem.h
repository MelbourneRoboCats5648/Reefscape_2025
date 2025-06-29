#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <Constants.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/PhotonCamera.h>



#include "subsystems/DriveSubsystem.h"

class VisionSubsystem : public frc2::SubsystemBase {
  public:
  VisionSubsystem(DriveSubsystem& driveSub);

  //Will be called periodically whenever the CommandScheduler runs.
  void Periodic() override;

  //Will be called periodically whenever the CommandScheduler runs during simulation.
  void SimulationPeriodic() override;

  private:
    photon::PhotonCamera camera{"photonvision"}; //check this name is what we gave it on the UI
    photon::PhotonPoseEstimator photonEstimator{
      VisionConstants::kTagLayout,
      photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
      VisionConstants::kRobotToCam};
    photon::PhotonPipelineResult m_latestResult;

    DriveSubsystem& m_drive;
};