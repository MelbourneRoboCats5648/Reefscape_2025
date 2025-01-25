#include "subsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem() {
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr ShooterSubsystem::ShooterSpeakerCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {
    m_motorShooterLeft.Set(speakerShooterSpeed);
    m_motorShooterRight.Set(-1.0*speakerShooterSpeed);})
   .FinallyDo([this]{ 
   m_motorShooterLeft.Set(0.0);
   m_motorShooterRight.Set(0.0);});
}

frc2::CommandPtr ShooterSubsystem::ShooterAmpCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] { 
     m_motorShooterLeft.Set(ampShooterSpeed);
     m_motorShooterRight.Set(-1.0*ampShooterSpeed);})
   .FinallyDo ([this]{
    m_motorShooterLeft.Set(0.0);
    m_motorShooterRight.Set(0.0);}); 
}

frc2::CommandPtr ShooterSubsystem::ShooterSpeakerAmpCommand() {
  return ShooterAmpCommand().WithTimeout(1.5_s).AndThen(ShooterSpeakerCommand().WithTimeout(1.5_s));

//stops all motors
void ShooterSubsystem::stopMotors() {
  m_motorShooterLeft.Set(0.0);
  m_motorShooterRight.Set(0.0);
}

void ShooterSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ShooterSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
