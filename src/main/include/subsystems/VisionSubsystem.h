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
  frc2::CommandPtr GetAprilTagID();
  frc2::CommandPtr GetTagTY();
  frc2::CommandPtr GetTagTX();

  
  

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

// The order of below is important as trigger needs boolean and boolean needs loop. 

 private:
//   frc::EventLoop m_aprilTagLoop;
//   frc::BooleanEvent m_aprilTagIDBoolean;
  
//  public:
//   frc2::Trigger m_aprilTagIDTrigger;

//  public: 
//   //KP Constants
//   double kReefAngularKP = 0.9;
//   double kReefForwardKP = 0.9;

};