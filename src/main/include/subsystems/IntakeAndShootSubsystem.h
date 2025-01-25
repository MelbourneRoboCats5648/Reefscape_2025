#pragma once

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

class IntakeAndShootSubsystem : public frc2::SubsystemBase {
 public:
  IntakeAndShootSubsystem(IntakeSubsystem& intakeSub,
    ShooterSubsystem& shooterSub);

  /**
   @param IntakeSubsystem The subsystem used by this command.
   @param ShooterSubsystem
   */

  frc2::CommandPtr PerformIntakeAndShootCommand();
  
  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  IntakeSubsystem& m_intakeSubsystem;
  ShooterSubsystem& m_shooterSubsystem;

};
