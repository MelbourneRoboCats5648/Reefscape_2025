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

frc::Trajectory VisionSubsystem::CreateTrajectory(frc::Pose2d targetPose) {
  // Set up config for trajectory
  frc::TrajectoryConfig config{DriveConstants::kMaxSpeed,
                               DriveConstants::kMaxAcceleration};
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.getDriveKinematics());
  // Apply the voltage constraint
  //config.AddConstraint(autoVoltageConstraint);  //CHECK THIS LATER

  auto currentPose = m_drive.getPoseEstimator().GetEstimatedPosition();

    // An example trajectory to follow.  All units in meters.
  auto traj = frc::TrajectoryGenerator::GenerateTrajectory(
      currentPose, //current pose from pose estimatior
      {}, //check this
      targetPose, //from the map
      // Pass the config
      config);

  return traj;
}

  frc2::CommandPtr VisionSubsystem::SwerveCommand(frc::Trajectory trajectory) {
    return frc2::SwerveControllerCommand<4>(
      trajectory, 
      [this] { return m_drive.GetPosition(); },
      m_drive.getDriveKinematics(),
      m_drive.getHolonomicController(),
      [this](std::array<frc::SwerveModuleState, 4> states) {
        m_drive.SetModuleStates(states);
      },
      {&m_drive}
      ).ToPtr();
  }

  // Reset odometry to the initial pose of the trajectory, run path following
  // command, then stop at the end.
  frc2::CommandPtr VisionSubsystem::Followtrajectory(frc::Trajectory trajectory) {
    return RunOnce([this, initialPose = trajectory.InitialPose()] {
               m_drive.getPoseEstimator().ResetPose(initialPose);  //I have no idea if its doing this reset right.
               })
      .AndThen(SwerveCommand(trajectory))
      .AndThen(
          frc2::cmd::RunOnce([this] { m_drive.StopAllModules(); }, {}));
  }

std::optional<frc::Pose2d> VisionSubsystem::GetPoseAtTag(const int& reefTagID) {
  if (poseMap.contains(reefTagID)) {
    std::cout << "ReefTagId:" << reefTagID << " has Pose2D" << std::endl;
    
    std::cout << "X at" << reefTagID << "=" << poseMap.at(reefTagID).X().value() << std::endl;
    std::cout << "Y at" << reefTagID << "=" << poseMap.at(reefTagID).Y().value() << std::endl;
    std::cout << "Z Rot at" << reefTagID << "=" << poseMap.at(reefTagID).Rotation().Degrees().value() << std::endl;

    return {poseMap.at(reefTagID)};
  }
  else {
    return {};
  }
  
}


void VisionSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
