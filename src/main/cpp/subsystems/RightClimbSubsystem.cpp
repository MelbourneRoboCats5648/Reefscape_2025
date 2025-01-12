#include "subsystems/RightClimbSubsystem.h"

RightClimbSubsystem::RightClimbSubsystem() {
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr RightClimbSubsystem::RightClimbUpCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_motorClimbRight.Set(climbUpSpeed); })
          .FinallyDo([this]{m_motorClimbRight.Set(0);});
}

frc2::CommandPtr RightClimbSubsystem::RightClimbDownCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_motorClimbRight.Set(climbDownSpeed);})
          .FinallyDo([this]{m_motorClimbRight.Set(0);});
}

void RightClimbSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void RightClimbSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

