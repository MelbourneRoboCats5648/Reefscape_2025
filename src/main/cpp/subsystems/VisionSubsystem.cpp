#include "subsystems/VisionSubsystem.h"

#include <frc2/command/DeferredCommand.h>

VisionSubsystem::VisionSubsystem(DriveSubsystem& driveSub)
  : m_drive(driveSub)
{
  photonEstimator.SetMultiTagFallbackStrategy(
    photon::PoseStrategy::LOWEST_AMBIGUITY);

  m_posePublisher = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("vision/pose").Publish();
  m_tagPublisher = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("vision/tag").Publish();
  m_destinationPublisher = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("vision/destination").Publish();
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
        auto pose = visionEst->estimatedPose.ToPose2d();
        m_posePublisher.Set(pose);
        auto stddevs = GetEstimationStdDevs(pose);
        m_drive.getPoseEstimator().AddVisionMeasurement(pose,
                                                        visionEst->timestamp,
                                                        {stddevs(0), stddevs(1), stddevs(2)});
      }
    }
  }
}

Eigen::Matrix<double, 3, 1> VisionSubsystem::GetEstimationStdDevs(frc::Pose2d estimatedPose) {
  Eigen::Matrix<double, 3, 1> estStdDevs =
      VisionConstants::kSingleTagStdDevs;
  auto targets = camera.GetLatestResult().GetTargets();
  int numTags = 0;
  units::meter_t avgDist = 0_m;
  for (const auto& tgt : targets) {
    auto tagPose =
        photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
    if (tagPose) {
      numTags++;
      avgDist += tagPose->ToPose2d().Translation().Distance(
          estimatedPose.Translation());
    }
  }
  if (numTags == 0) {
    return estStdDevs;
  }
  avgDist /= numTags;
  if (numTags > 1) {
    estStdDevs = VisionConstants::kMultiTagStdDevs;
  }
  if (numTags == 1 && avgDist > 4_m) {
    estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::max())
                      .finished();
  } else {
    estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
  }
  return estStdDevs;
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

  // Reset odometry to the initial pose of the trajectory, run path following
  // command, then stop at the end.
  frc2::CommandPtr VisionSubsystem::Followtrajectory(frc::Trajectory trajectory) {
    return RunOnce([this, initialPose = trajectory.InitialPose()] {
               m_drive.getPoseEstimator().ResetPose(initialPose);  //I have no idea if its doing this reset right.
               })
      .AndThen(
        frc2::SwerveControllerCommand<4>(
              trajectory, 
              [this] { return m_drive.GetPosition(); },
              m_drive.getDriveKinematics(),
              m_drive.getHolonomicController(),
              [this](std::array<frc::SwerveModuleState, 4> states) {
                m_drive.SetModuleStates(states);
              },
              {&m_drive}
              ).ToPtr()
      )
      .FinallyDo([this] { m_drive.StopAllModules(); });
  }

std::optional<frc::Pose2d> VisionSubsystem::GetPoseAtTag(const int& reefTagID) {
  // if (poseMap.contains(reefTagID)) {
  //   std::cout << "ReefTagId:" << reefTagID << " has Pose2D" << std::endl;
    
  //   std::cout << "X at" << reefTagID << "=" << poseMap.at(reefTagID).X().value() << std::endl;
  //   std::cout << "Y at" << reefTagID << "=" << poseMap.at(reefTagID).Y().value() << std::endl;
  //   std::cout << "Z Rot at" << reefTagID << "=" << poseMap.at(reefTagID).Rotation().Degrees().value() << std::endl;

  //   return {poseMap.at(reefTagID)};
  // }
  // else {
  //   return {};
  // }
  
  if (!VisionConstants::kReefTagIDs.contains(reefTagID)) return {};
  return {VisionConstants::kTagLayout.GetTagPose(reefTagID).value().ToPose2d()};
}

 frc2::CommandPtr VisionSubsystem::MoveToTarget(ReefPosition position) {
  return frc2::DeferredCommand([this, position]{
    auto latestResult = camera.GetLatestResult();
    if (latestResult.HasTargets()){
      int targetID = latestResult.GetBestTarget().GetFiducialId(); 
      std::optional<frc::Pose2d> targetPose = GetPoseAtTag(targetID);
      if (targetPose) {
        m_tagPublisher.Set(targetPose.value());
        frc::Pose2d transformedPose;
        switch (position) {
          case ReefPosition::Left: 
          {
            transformedPose = GetLeftPose(targetPose.value());
            break;
          }
          case ReefPosition::Right: 
          {
            transformedPose = GetRightPose(targetPose.value());
            break;
          }
        }
        frc::Trajectory trajectory = CreateTrajectory(transformedPose);
        m_destinationPublisher.Set(transformedPose);
        return Followtrajectory(trajectory);
      }
    }

    return RunOnce([this]{}); // do nothing
  }, {&m_drive}).ToPtr();
}

frc::Pose2d VisionSubsystem::GetLeftPose(const frc::Pose2d& tagPose) {
  return tagPose.TransformBy(frc::Transform2d(
    frc::Translation2d(VisionConstants::kTagDistance, -VisionConstants::kTagSideOffset),
    frc::Rotation2d(180.0_deg) // face the tag
  ));
}

frc::Pose2d VisionSubsystem::GetRightPose(const frc::Pose2d& tagPose) {
  return tagPose.TransformBy(frc::Transform2d(
    frc::Translation2d(VisionConstants::kTagDistance, VisionConstants::kTagSideOffset),
    frc::Rotation2d(180.0_deg) // face the tag
  ));
}

void VisionSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
