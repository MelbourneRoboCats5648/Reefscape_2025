#include "subsystems/ArmSubsystem.h"

#include <rev/SparkMax.h>

#include <rev/config/SparkMaxConfig.h>

using namespace ArmConstants;

ArmSubsystem::ArmSubsystem() {
  // Implementation of subsystem constructor goes here.
  rev::spark::SparkMaxConfig armMotorConfig;

  //Set parameters that will apply to elevator motor.
    armMotorConfig.SmartCurrentLimit(50).SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake); 
    armMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);

      //armMotorConfig.limitSwitch
      //  .ReverseLimitSwitchType(rev::spark::LimitSwitchConfig::Type::kNormallyOpen)
      //  .ReverseLimitSwitchEnabled(true);

      armMotorConfig.softLimit
        .ForwardSoftLimit(ArmConstants::extendSoftLimit.value())
        .ForwardSoftLimitEnabled(true);

      armMotorConfig.softLimit
        .ReverseSoftLimit(ArmConstants::retractSoftLimit.value())
        .ReverseSoftLimitEnabled(true);

  //PID Controller 
  m_controller.SetTolerance(units::turn_t(ArmConstants::kArmPositionToleranceTurns), units::turns_per_second_t(ArmConstants::kArmVelocityTolerancePerSecond));

  armMotorConfig.encoder.PositionConversionFactor(ArmConstants::gearRatio).VelocityConversionFactor(ArmConstants::gearRatio);

  /* Apply the configuration to the SPARKs.
   *
   * kResetSafeParameters is used to get the SPARK MAX to a known state. This
   * is useful in case the SPARK MAX is replaced.
   *
   * kPersistParameters is used to ensure the configuration is not lost when
   * the SPARK MAX loses power. This is useful for power cycles that may occur
   * mid-operation. */
   m_armMotor.Configure(armMotorConfig,
                        rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                        rev::spark::SparkMax::PersistMode::kPersistParameters);

  ResetEncoder();
  SetDefaultCommand(HoldPositionCommand());
}

units::turn_t ArmSubsystem::GetArmAngle() {
  return units::turn_t(m_armEncoder.GetPosition());
}

bool ArmSubsystem::IsGoalReached() {
  return m_controller.AtGoal();
}

void ArmSubsystem::MoveArm(double speed) {
  // Inline construction of command goes here.
  m_armMotor.Set(speed);
}

frc2::CommandPtr ArmSubsystem::MoveUpCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_armMotor.Set(-0.1); });
}

frc2::CommandPtr ArmSubsystem::MoveDownCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_armMotor.Set(0.1); });
}

frc2::CommandPtr ArmSubsystem::MoveToAngleCommand(units::turn_t goal) {
  // Inline construction of command goes here. 
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return
    StartRun(
      [this, goal] { m_controller.SetGoal(goal); },
      [this] { SetpointControlCommand(); }
    )
    .Until([this] { return IsGoalReached(); });

                        // m_closedLoopController.SetReference(m_armGoal.position.value(), 
                        //                         rev::spark::SparkLowLevel::ControlType::kPosition,
                        //                         rev::spark::kSlot0,
                        //                         m_armFeedforward.Calculate(m_armSetpoint.velocity).value());
}



//To move down supply a negative
frc2::CommandPtr ArmSubsystem::RotateBy(units::turn_t angle) {
      units::turn_t moveGoal = (GetArmAngle() + angle);
      return MoveToAngleCommand(moveGoal);
}

//stops motor
void ArmSubsystem::StopMotor() {
  m_armMotor.Set(0.0);
}

//reset encoder
void ArmSubsystem::ResetEncoder() {
  m_armEncoder.SetPosition(ArmConstants::resetEncoder.value());
}

void ArmSubsystem::SetpointControl() {
  units::turn_t position = GetArmAngle();
  double output = 
    m_controller.Calculate(position) // profiled PID
    + (m_armFeedforward.Calculate(position, m_controller.GetSetpoint().velocity) / frc::RobotController::GetBatteryVoltage()); // feedforward, normalised
  output = std::clamp(output, -ArmConstants::maxOutput, ArmConstants::maxOutput); // clamp to [-maxOutput, maxOutput]

  m_armMotor.Set(output);
}

frc2::CommandPtr ArmSubsystem::SetpointControlCommand() {
  return Run([this] {
    SetpointControl();
  });
}

frc2::CommandPtr ArmSubsystem::HoldPositionCommand() {
  return StartRun(
    [this] { m_controller.SetGoal(GetArmAngle()); },
    [this] { SetpointControl(); }
  );
}

void ArmSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  frc::SmartDashboard::PutNumber("armEncoderValue", GetArmAngle().value());  
}

void ArmSubsystem::OnLimitSwitchActivation() {
      if(m_limitSwitchArm.Get()) {
      StopMotor();
      ResetEncoder();
      }
}

void ArmSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
