#include "subsystems/IntakeAndShootSubsystem.h"

IntakeAndShootSubsystem::IntakeAndShootSubsystem() {
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr IntakeAndShootSubsystem::IntakeAndShootCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] { 
        m_intakeSubsystem.EjectCommand()
        .AndThen(m_shootersubsystem.ShooterAmpCommand());
    }).FinallyDo([this]{
        m_intakeSubsystem.getIntakeWheel().Set(0);
        m_shooterSubsystem.getmotorShooterLeft().Set(0);
        m_shooterSubSystem.getmotorShooterRight().Set(0);
    });
}