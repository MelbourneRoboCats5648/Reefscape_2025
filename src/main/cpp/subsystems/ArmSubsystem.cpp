#include "subsystems/ArmSubsystem.h"

#include <rev/SparkMax.h>

#include <rev/config/SparkMaxConfig.h>

ArmSubsystem::ArmSubsystem() {
  // Implementation of subsystem constructor goes here.
  rev::spark::SparkMaxConfig elevatorArmMotorConfig;

  /*
   * Set parameters that will apply to elevator motor.
   */
   elevatorArmMotorConfig.SmartCurrentLimit(50).SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake); 
   //fixme - test to find out the current limit

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
   m_elevatorArmMotor.Configure(elevatorArmMotorConfig,
                        rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                        rev::spark::SparkMax::PersistMode::kPersistParameters);
}

frc2::CommandPtr ArmSubsystem::MoveArmUpCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_elevatorArmMotor.Set(-0.1);})
          .FinallyDo([this]{m_elevatorArmMotor.Set(0);});
}

frc2::CommandPtr ArmSubsystem::MoveArmDownCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_elevatorArmMotor.Set(0.1);})
          .FinallyDo([this]{m_elevatorArmMotor.Set(0);});
}

frc2::CommandPtr ArmSubsystem::MoveArmToLevelCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_elevatorArmMotor.Set(-0.2);})
          .FinallyDo([this]{m_elevatorArmMotor.Set(0);});
}

//stops motor
void ArmSubsystem::StopMotor() {
  m_elevatorArmMotor.Set(0.0);
}

void ArmSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ArmSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
