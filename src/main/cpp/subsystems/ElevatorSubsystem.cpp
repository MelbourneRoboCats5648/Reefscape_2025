#include "subsystems/ElevatorSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/motorcontrol/PWMSparkMax.h>

ElevatorSubsystem::ElevatorSubsystem() {
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr ElevatorSubsystem::GoUpToL1Command() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_motorController.Set(-0.3); })
          .FinallyDo([this]{m_motorController.Set(0);});
}

frc2::CommandPtr ElevatorSubsystem::GoUpToL2Command() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_motorController.Set(-0.5);})
          .FinallyDo([this]{m_motorController.Set(0);});
}

frc2::CommandPtr ElevatorSubsystem::GoUpToL3Command() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_motorController.Set(-0.7);})
          .FinallyDo([this]{m_motorController.Set(0);});
}

// the values set is for motor output (need to test) 
void ElevatorSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ElevatorSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

