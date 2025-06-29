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
  std::cout << "AIMING AND RANGING" << std::endl;

  //so this will be called by a button.
  bool targetVisible = false;
  units::degree_t targetYaw = 0.0_deg;
  units::meter_t targetRange = 0.0_m;
  std::set<int> aprilTagSet = {17, 18, 19, 20, 21, 22}; // to be changed

  //auto results = camera.GetAllUnreadResults();
  //std::cout << "CAMERA NAME = " << camera.GetCameraName() << std::endl;
  //std::cout << "RESULT SIZE = " << results.size() << std::endl;
  
  auto latestResult = camera.GetLatestResult();
  if (latestResult.HasTargets())
  {
    std::cout << "LATEST RESULT HAS TARGET!!" << std::endl;
      auto target = latestResult.GetBestTarget();
        if (aprilTagSet.contains(target.GetFiducialId())) 
        {
          std::cout << "TARGET HAS ID = " << target.GetFiducialId() << std::endl;

          // Found Tag in the targeted apriltage set, record its information
          targetYaw = units::degree_t{target.GetYaw()};
          targetRange = photon::PhotonUtils::CalculateDistanceToTarget(
              0.2_m,      // height of camera
              1_m,    // height of april tage
              -30.0_deg,  // camera pitch angle
              units::degree_t{target.GetPitch()});
          targetVisible = true;
        }
    }
  
  units::meters_per_second_t xSpeed = 0_mps;
  units::meters_per_second_t ySpeed = 0_mps;
  units::radians_per_second_t rotSpeed = 0_rad_per_s;

    // Auto-align
  if (targetVisible) 
  {
    std::cout << "TARGET VISIBLE" << std::endl;

    // Driver wants auto-alignment to tag x
    // And, tag x is in sight, so we can turn toward it.
    // Override the driver's turn command with an automatic one that turns
    // toward the tag and gets the range right.
    units::angle::degree_t angleError = reefDesiredAngle - targetYaw;
    rotSpeed = angleError.value() * visionTurnKP * DriveConstants::kMaxAngularSpeed;

    units::meter_t distError = reefDesiredRange - targetRange;
    xSpeed = distError.value() * visionStrafeKP * DriveConstants::kMaxSpeed;

    std::cout << "angleError = " << angleError.value() << std::endl;
    std::cout << "distError = " << distError.value() << std::endl;
  }

  // Command drivetrain motors based on target speeds
  std::cout << "xSpeed = " << xSpeed.value() << std::endl;
  std::cout << "ySpeed = " << ySpeed.value() << std::endl;
  std::cout << "rotSpeed = " << rotSpeed.value() << std::endl;

  //rotSpeed = m_drive.m_rotLimiter.Calculate(rotSpeed);
  //xSpeed = m_drive.m_xLimiter.Calculate(xSpeed);
  //ySpeed = m_drive.m_yLimiter.Calculate(ySpeed);

  std::cout << "xSpeedLim = " << xSpeed.value() << std::endl;
  std::cout << "ySpeedLim = " << ySpeed.value() << std::endl;
  std::cout << "rotSpeedLim = " << rotSpeed.value() << std::endl;

  m_drive.Drive(xSpeed, ySpeed, rotSpeed);
}


void VisionSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
