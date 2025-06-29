#include "subsystems/VisionSubsystem.h"

VisionSubsystem::VisionSubsystem(DriveSubsystem& driveSub)
  : m_drive(driveSub)
{
  photonEstimator.SetMultiTagFallbackStrategy(
    photon::PoseStrategy::LOWEST_AMBIGUITY);
}

void VisionSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
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
       if (result.HasTargets()) {
         // At least one AprilTag was seen by the camera
       for (auto& targets: result.GetTargets()) {
        if (targets.GetFiducialId() == 19) {
          // Found Tag 19, record its information -testing
          double targetYaw = 0.0;
          targetYaw= targets.GetYaw();
          frc::SmartDashboard::PutNumber("Vision/fiducial19_yaw", targetYaw);  
        }
    }
}

void VisionSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
