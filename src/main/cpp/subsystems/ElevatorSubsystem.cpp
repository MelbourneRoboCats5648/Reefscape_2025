#include "subsystems/ElevatorSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>

#include <rev/config/SparkBaseConfig.h>
#include <frc2/command/button/CommandXboxController.h>

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
    /**
   * Get forward and rotation values from the joystick. Invert the joystick's
   * Y value because its forward direction is negative.
   */
  double forward = - xboxController.GetLeftY();
  double rotation = xboxController.GetRightX();

  /*
   * Apply values to left and right side. We will only need to set the leaders
   * since the other motors are in follower mode.
   */
   m_elevatorLiftMotor.Set(forward + rotation);
   m_elevatorLiftMotor.Set(forward - rotation);

  // Display the applied output of the left and right side onto the dashboard
  frc::SmartDashboard::PutNumber("Left Out", m_elevatorLiftMotor.GetAppliedOutput());
  frc::SmartDashboard::PutNumber("Right Out", m_elevatorLiftMotor.GetAppliedOutput());
}

void ElevatorSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
