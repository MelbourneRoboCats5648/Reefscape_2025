#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem() {
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr IntakeSubsystem::CollectCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] { 
        m_motorIntakeWheel.Set(intakeWheelInSpeed);
    }).FinallyDo([this]{
        m_motorIntakeWheel.Set(0);
    });
}

frc2::CommandPtr IntakeSubsystem::EjectCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {
        m_motorIntakeWheel.Set(intakeWheelOutSpeed);
    }).FinallyDo([this]{
        m_motorIntakeWheel.Set(0);
    });
}

frc2::CommandPtr IntakeSubsystem::RetractCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {
        m_motorIntakeArm.Set(intakeArmRetractSpeed);
    }).FinallyDo([this]{
        m_motorIntakeWheel.Set(0);
    });
}

frc2::CommandPtr IntakeSubsystem::ExtendCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {
        m_motorIntakeArm.Set(intakeArmExtendSpeed);
    }).FinallyDo([this]{
        m_motorIntakeWheel.Set(0);
    });
}


void IntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void IntakeSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
