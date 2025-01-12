#include "subsystems/LeftClimbSubsystem.h"

LeftClimbSubsystem::LeftClimbSubsystem() {
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr LeftClimbSubsystem::LeftClimbUpCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_motorClimbLeft.Set(climbUpSpeed);})
          .FinallyDo([this]{m_motorClimbLeft.Set(0);});
}

frc2::CommandPtr LeftClimbSubsystem::LeftClimbDownCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_motorClimbLeft.Set(climbDownSpeed);})
          .FinallyDo([this]{m_motorClimbLeft.Set(0);});
}

void LeftClimbSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void LeftClimbSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
