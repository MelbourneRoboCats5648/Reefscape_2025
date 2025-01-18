#include "subsystems/ElevatorSubsystem.h"

#include <rev/SparkMax.h>

#include <rev/config/SparkBaseConfig.h>

ElevatorSubsystem::ElevatorSubsystem() {
  // Implementation of subsystem constructor goes here.
  rev::spark::SparkBaseConfig elevatorMotorConfig;

  /*
   * Set parameters that will apply to elevator motor.
   */
   elevatorMotorConfig.SmartCurrentLimit(50).SetIdleMode(SparkBaseConfig::IdleMode::kBrake);

  /*
   * Apply the configuration to the SPARKs.
   *
   * kResetSafeParameters is used to get the SPARK MAX to a known state. This
   * is useful in case the SPARK MAX is replaced.
   *
   * kPersistParameters is used to ensure the configuration is not lost when
   * the SPARK MAX loses power. This is useful for power cycles that may occur
   * mid-operation.
   */
   m_elevatorLiftMotor.Configure(elevatorMotorConfig,
                        SparkMax::ResetMode::kResetSafeParameters,
                        SparkMax::PersistMode::kPersistParameters);
}

frc2::CommandPtr ElevatorSubsystem::MoveUpToL1Command() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_elevatorLiftMotor.Set(-0.3);})
          .FinallyDo([this]{m_elevatorLiftMotor.Set(0);});
}

frc2::CommandPtr ElevatorSubsystem::MoveUpToL2Command() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_elevatorLiftMotor.Set(-0.5);})
          .FinallyDo([this]{m_elevatorLiftMotor.Set(0);});
}

frc2::CommandPtr ElevatorSubsystem::MoveUpToL3Command() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_elevatorLiftMotor.Set(-0.7);})
          .FinallyDo([this]{m_elevatorLiftMotor.Set(0);});
}

frc2::CommandPtr ElevatorSubsystem::MoveDownCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_elevatorLiftMotor.Set(0.3);})
          .FinallyDo([this]{m_elevatorLiftMotor.Set(0);});
}

void ElevatorSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ElevatorSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
