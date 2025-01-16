#pragma once

#include "LimelightHelpers.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>

class VisionSubsystem : public frc2::SubsystemBase {
 public:
  VisionSubsystem();

  /** 
   * Example command factory method.
   */
  frc2::CommandPtr AimRobotToTargetReef();
  
  

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:

};

