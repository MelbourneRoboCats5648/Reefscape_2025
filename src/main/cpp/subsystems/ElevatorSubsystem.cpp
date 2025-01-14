#include "subsystems/ElevatorSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/motorcontrol/PWMSparkMax.h>

#include "subsystems/RightClimbSubsystem.h"

RightClimbSubsystem::RightClimbSubsystem() {
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr RightClimbSubsystem::RightClimbUpCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_motorClimbRight.Set(rightClimbUpSpeed); })
          .FinallyDo([this]{m_motorClimbRight.Set(0);});
}

frc2::CommandPtr RightClimbSubsystem::RightClimbDownCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_motorClimbRight.Set(rightClimbDownSpeed);})
          .FinallyDo([this]{m_motorClimbRight.Set(0);});
}

void RightClimbSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void RightClimbSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

