#include "subsystems/IntakeAndShootSubsystem.h"

IntakeAndShootSubsystem::IntakeAndShootSubsystem(IntakeSubsystem& intakeSub, ShooterSubsystem& shooterSub) 
  : m_intakeSubsystem(intakeSub),
    m_shooterSubsystem(shooterSub)
{
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr IntakeAndShootSubsystem::IntakeAndShootCommand() {
  return Run([this] { 
        m_intakeSubsystem.EjectCommand()
        .AndThen(m_shooterSubsystem.ShooterAmpCommand());
    }).FinallyDo([this]{
        m_intakeSubsystem.stopMotors();
        m_shooterSubsystem.stopMotors();
    });
}

