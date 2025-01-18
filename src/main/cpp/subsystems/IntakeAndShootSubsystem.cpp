#include "subsystems/IntakeAndShootSubsystem.h"

IntakeAndShootSubsystem::IntakeAndShootSubsystem(IntakeSubsystem& intakeSub, ShooterSubsystem& shooterSub) 
  : m_intakeSubsystem(intakeSub),
    m_shooterSubsystem(shooterSub)
{

}
frc2::CommandPtr IntakeAndShootSubsystem::PerformIntakeAndShootCommand() {
  return Run([this] { m_intakeSubsystem.EjectCommand();})
  .AndThen([this] {m_shooterSubsystem.ShooterAmpCommand();})
  .FinallyDo([this]{
    m_intakeSubsystem.stopMotors();
    m_shooterSubsystem.stopMotors(); });
}

void IntakeAndShootSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void IntakeAndShootSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}