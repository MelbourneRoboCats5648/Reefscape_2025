#include "subsystems/ClimbSubsystem.h"

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

using namespace ClimbConstants;

ClimbSubsystem::ClimbSubsystem() {
  // Implementation of subsystem constructor goes here.
  rev::spark::SparkMaxConfig climbMotorConfig;

  //Set parameters that will apply to elevator motor.
  climbMotorConfig.SmartCurrentLimit(50);
  climbMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);

  climbMotorConfig.limitSwitch
    .ForwardLimitSwitchEnabled(false)
    .ReverseLimitSwitchEnabled(false);

  climbMotorConfig.softLimit
    .ForwardSoftLimit(ClimbConstants::extendSoftLimit.value())
    .ForwardSoftLimitEnabled(true);

  climbMotorConfig.softLimit
    .ReverseSoftLimit(ClimbConstants::retractSoftLimit.value())
    .ReverseSoftLimitEnabled(true);

  climbMotorConfig.encoder.PositionConversionFactor(ClimbConstants::gearRatio).VelocityConversionFactor(ClimbConstants::gearRatio);
  
  m_climbMotor.Configure(climbMotorConfig,
                        rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                        rev::spark::SparkMax::PersistMode::kPersistParameters);

  ResetEncoder();
  LockRatchet();
}

units::turn_t ClimbSubsystem::GetClimbAngle() {
  return units::turn_t(m_climbEncoder.GetPosition());
}

units::turns_per_second_t ClimbSubsystem::GetClimbVelocity() {
  return units::turns_per_second_t(m_climbEncoder.GetVelocity() / 60.0);
}

bool ClimbSubsystem::IsGoalReached() {
  return m_controller.AtGoal();
}

void ClimbSubsystem::MoveClimb(double speed) {
  // Inline construction of command goes here.
  m_climbMotor.Set(speed);
}

frc2::CommandPtr ClimbSubsystem::MoveUpCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_climbMotor.Set(-0.5); })
          .FinallyDo([this]{ m_climbMotor.Set(0); });
}

frc2::CommandPtr ClimbSubsystem::MoveDownCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_climbMotor.Set(0.5); })
          .FinallyDo([this]{ m_climbMotor.Set(0); });
}

void ClimbSubsystem::SetpointControl() {
  units::turn_t position = GetClimbAngle();
  double output = 
    m_controller.Calculate(position) // profiled PID
    + (m_climbFeedforward.Calculate(position, m_controller.GetSetpoint().velocity) / frc::RobotController::GetBatteryVoltage()); // feedforward, normalised
  output = std::clamp(output, -ClimbConstants::maxOutput, ClimbConstants::maxOutput); // clamp to [-maxOutput, maxOutput]

  m_climbMotor.Set(output);
}

frc2::CommandPtr ClimbSubsystem::MoveToAngleCommand(units::turn_t goal) {
  // Inline construction of command goes here. 
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return ReleaseClimbCommand()
  .AndThen(
    StartRun(
      [this, goal] { m_controller.Reset(GetClimbAngle(), GetClimbVelocity()); m_controller.SetGoal(goal); },
      [this] { SetpointControl(); }
    )
    .Until([this] { return IsGoalReached(); })
  )
  .FinallyDo([this] {
    LockRatchet();
  });

}

//stops motor
void ClimbSubsystem::StopMotor() {
  m_climbMotor.Set(0.0);
}

void ClimbSubsystem::LockRatchet(){
  ratchetServo.Set(ClimbConstants::lockValue);  // issue 119 - Check that this is lock
}

void ClimbSubsystem::ReleaseRatchet(){
  ratchetServo.Set(ClimbConstants::releaseValue);  // issue 119 - Check that this is lock
}

frc2::CommandPtr ClimbSubsystem::LockClimbCommand()
{
  return Run([this] {
    StopMotor();
    LockRatchet();
  }).WithTimeout(0.5_s);
}

frc2::CommandPtr ClimbSubsystem::ReleaseClimbCommand()
{
  return Run([this] {
    ReleaseRatchet();
  }).WithTimeout(0.5_s);
}

//reset encoder
void ClimbSubsystem::ResetEncoder() {
  m_climbEncoder.SetPosition(ClimbConstants::resetEncoder.value());
}

void ClimbSubsystem::Periodic() {// Implementation of subsystem periodic method goes here.
  auto setpoint = m_controller.GetSetpoint();
  frc::SmartDashboard::PutNumber("Climb/setpoint/position", setpoint.position.value());
  frc::SmartDashboard::PutNumber("Climb/setpoint/velocity", setpoint.velocity.value());

  auto goal = m_controller.GetGoal();
  frc::SmartDashboard::PutNumber("Climb/goal/position", goal.position.value());
  frc::SmartDashboard::PutNumber("Climb/goal/velocity", goal.velocity.value());

  frc::SmartDashboard::PutNumber("Climb/encoder/position", GetClimbAngle().value());
  frc::SmartDashboard::PutNumber("Climb/encoder/velocity", GetClimbVelocity().value());
}

void ClimbSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
