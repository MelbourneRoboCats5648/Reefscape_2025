#include "subsystems/VisionSubsystem.h"

VisionSubsystem::VisionSubsystem(DriveSubsystem& driveSub)
  : m_drive(driveSub)
{
  photonEstimator.SetMultiTagFallbackStrategy(
    photon::PoseStrategy::LOWEST_AMBIGUITY);
}

void VisionSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  {
    // Run each new pipeline result through our pose estimator
    for (const auto& result : camera.GetAllUnreadResults()) {
      // cache result and update pose estimator
      auto visionEst = photonEstimator.Update(result);
      m_latestResult = result;

      if (visionEst) {
        m_drive.getPoseEstimator().AddVisionMeasurement(visionEst->estimatedPose.ToPose2d(),
                                                        visionEst->timestamp);
                                     //GetEstimationStdDevs(visionEst->estimatedPose.ToPose2d()));
      }
    }
  }
}

void VisionSubsystem::AimAndRange() {
  //so this will be called by a button.
  bool targetVisible = false;
  units::degree_t targetYaw = 0.0_deg;
  units::meter_t targetRange = 0.0_m;
  std::set<int> aprilTagSet = {1, 2, 3, 4, 5}; // to be changed

  auto results = camera.GetAllUnreadResults();
  if (results.size() > 0) 
  {
    // Camera processed a new frame since last
    // Get the last one in the list.
    auto result = results[results.size() - 1];
    if (result.HasTargets()) 
    {
      // At least one AprilTag was seen by the camera
      //for (auto& target : result.GetTargets()) 
      auto target = result.GetBestTarget();
      //{
        if (aprilTagSet.contains(target.GetFiducialId())) 
        {
          // Found Tag in the targeted apriltage set, record its information
          targetYaw = units::degree_t{target.GetYaw()};
          targetRange = photon::PhotonUtils::CalculateDistanceToTarget(
              0.5_m,      // height of camera
              1.435_m,    // height of april tage
              -30.0_deg,  // camera pitch angle
              units::degree_t{target.GetPitch()});
          targetVisible = true;
        }
      //}
    }
  }
  units::meters_per_second_t xSpeed = 0_mps;
  units::meters_per_second_t ySpeed = 0_mps;
  units::radians_per_second_t rotSpeed = 0_rad_per_s;

    // Auto-align
  if (targetVisible) 
  {
    // Driver wants auto-alignment to tag x
    // And, tag x is in sight, so we can turn toward it.
    // Override the driver's turn command with an automatic one that turns
    // toward the tag and gets the range right.
    rotSpeed = (reefDesiredAngle - targetYaw).value() * visionTurnKP *
           DriveConstants::kMaxAngularSpeed;
    xSpeed = (reefDesiredRange - targetRange).value() * visionStrafeKP *
              DriveConstants::kMaxSpeed; //FIXME CHECK THIS IS RIGHT CONSTANT
  }

  // Command drivetrain motors based on target speeds
 
  m_drive.Drive(xSpeed, ySpeed, rotSpeed);

}

void VisionSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
