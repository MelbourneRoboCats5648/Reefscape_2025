#include "subsystems/ElevatorSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/motorcontrol/PWMSparkMax.h>

#include <rev/config/SparkMaxConfig.h>
#include <frc/Joystick.h>

ElevatorSubsystem::ElevatorSubsystem() {
  // Implementation of subsystem constructor goes here.
  rev::spark::SparkMaxConfig globalConfig;
  rev::spark::SparkMaxConfig rightLeaderConfig;
  rev::spark::SparkMaxConfig leftFollowerConfig;
  rev::spark::SparkMaxConfig rightFollowerConfig;

  /*
   * Set parameters that will apply to all SPARKs. We will also use this as
   * the left leader config.
   */
  SparkMaxConfig globalConfig.SmartCurrentLimit(50).SetIdleMode(SparkMaxConfig::IdleMode::kBrake);

  // Apply the global config and invert since it is on the opposite side
  SparkMaxConfig rightLeaderConfig.Apply(globalConfig).Inverted(true);

  // Apply the global config and set the leader SPARK for follower mode
  SparkMaxConfig leftFollowerConfig.Apply(globalConfig).Follow(m_leftLeader);

  // Apply the global config and set the leader SPARK for follower mode
  SparkMaxConfig rightFollowerConfig.Apply(globalConfig).Follow(m_rightLeader);

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
  SparkMaxConfig m_leftLeader.Configure(globalConfig,
                        SparkMax::ResetMode::kResetSafeParameters,
                        SparkMax::PersistMode::kPersistParameters);
  SparkMaxConfig m_rightLeader.Configure(rightLeaderConfig,
                        SparkMax::ResetMode::kResetSafeParameters,
                        SparkMax::PersistMode::kPersistParameters);
}

 ElevatorSubsystem::ElevatorSubsystem() {
  // Display the applied output of the left and right side onto the dashboard
  frc::SmartDashboard::PutNumber("Left Out", m_leftLeader.GetAppliedOutput());
  frc::SmartDashboard::PutNumber("Right Out", m_rightLeader.GetAppliedOutput());
}
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {

  /**
   * Get forward and rotation values from the joystick. Invert the joystick's
   * Y value because its forward direction is negative.
   */
  double forward = - joystick.GetLeftY();
  double rotation = joystick.GetRightX();

  /*
   * Apply values to left and right side. We will only need to set the leaders
   * since the other motors are in follower mode.
   */
  rev::spark::SparkMaxConfig m_leftLeader.Set(forward + rotation);
  rev::spark::SparkMaxConfig m_rightLeader.Set(forward - rotation);
}

frc2::CommandPtr ElevatorSubsystem::GoUpToL1Command() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_motorController.Set(-0.3);})
          .FinallyDo([this]{m_motorController.Set(0);});
}

frc2::CommandPtr ElevatorSubsystem::GoUpToL2Command() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_motorController.Set(-0.5);})
          .FinallyDo([this]{m_motorController.Set(0);});
}

frc2::CommandPtr ElevatorSubsystem::GoUpToL3Command() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_motorController.Set(-0.7);})
          .FinallyDo([this]{m_motorController.Set(0);});
}

// the values set is for motor output (need to test) 

void ElevatorSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ElevatorSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

