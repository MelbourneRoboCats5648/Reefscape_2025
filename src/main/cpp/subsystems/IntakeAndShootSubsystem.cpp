#include "subsystems/IntakeAndShootSubsystem.h"

IntakeAndShootSubsystem::IntakeAndShootSubsystem(IntakeSubsystem& intakeSub, ShooterSubsystem& shooterSub) 
  : m_intakeSubsystem(intakeSub),
    m_shooterSubsystem(shooterSub)
{

}
frc2::CommandPtr IntakeAndShootSubsystem::PerformIntakeAndShootCommand() {
  return m_intakeSubsystem.ExtendCommand().WithTimeout(1.5_s)
  //.AndThen(m_intakeSubsystem.CollectCommand())
  // m_intakeSubsystem.RetractCommand()
  // m_intakeSubsystem.EjectCommand()
  .AndThen(m_shooterSubsystem.ShooterAmpCommand().WithTimeout(1.5_s))
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