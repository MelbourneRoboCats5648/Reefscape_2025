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
        auto targets = result.GetTargets();
        frc::SmartDashboard::PutNumber("Vision/fiducial", targets);

      
    }
}

void VisionSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
