#include "subsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem() {
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr ShooterSubsystem::ShooterAmpCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([/* this */] { /* one-time action goes here */ });
}

frc2::CommandPtr ShooterSubsystem::ShooterSpeakerCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([/* this */] { /* one-time action goes here */ });
}



void ShooterSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ShooterSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
