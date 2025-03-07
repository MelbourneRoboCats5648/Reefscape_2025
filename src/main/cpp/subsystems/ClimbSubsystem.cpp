#include "subsystems/ClimbSubsystem.h"

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

using namespace ClimbConstants;

ClimbSubsystem::ClimbSubsystem() {
  // Implementation of subsystem constructor goes here.
  rev::spark::SparkMaxConfig climbMotorConfig;

  //Set parameters that will apply to elevator motor.
    climbMotorConfig.SmartCurrentLimit(50).SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake); 
    climbMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);

      climbMotorConfig.softLimit
        .ForwardSoftLimit(ClimbConstants::extendSoftLimit.value())
        .ForwardSoftLimitEnabled(true);

      climbMotorConfig.softLimit
        .ReverseSoftLimit(ClimbConstants::retractSoftLimit.value())
        .ReverseSoftLimitEnabled(true);

  //PID Controller 
  /* Configure the closed loop controller. We want to make sure we set the
  * feedback sensor as the primary encoder. */
  climbMotorConfig.closedLoop
     .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
    .P(ClimbConstants::kP)
    .I(ClimbConstants::kI)
    .D(ClimbConstants::kD)
    .OutputRange(-ClimbConstants::maxOutput, ClimbConstants::maxOutput);

   climbMotorConfig.encoder.PositionConversionFactor(ClimbConstants::gearRatio).VelocityConversionFactor(ClimbConstants::gearRatio);

  /* Apply the configuration to the SPARKs.
   *
   * kResetSafeParameters is used to get the SPARK MAX to a known state. This
   * is useful in case the SPARK MAX is replaced.
   *
   * kPersistParameters is used to ensure the configuration is not lost when
   * the SPARK MAX loses power. This is useful for power cycles that may occur
   * mid-operation. */
   m_climbMotor.Configure(climbMotorConfig,
                        rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                        rev::spark::SparkMax::PersistMode::kPersistParameters);

  ResetEncoder();
  LockRatchet();
}

void ClimbSubsystem::UpdateSetpoint() {  
  m_climbSetpoint.position = GetClimbAngle();
  m_climbSetpoint.velocity = 0.0_tps; 
}

units::turn_t ClimbSubsystem::GetClimbAngle() {
  return units::turn_t(m_climbEncoder.GetPosition());
}

frc::TrapezoidProfile<units::turn>::State& ClimbSubsystem::GetSetpoint() {
  return  m_climbSetpoint;
} 

frc::TrapezoidProfile<units::turn>::State& ClimbSubsystem::GetGoal() {
  return  m_climbGoal;
} 

bool ClimbSubsystem::IsGoalReached() {
  double errorPosition = std::abs(GetSetpoint().position.value() - GetGoal().position.value());
  double errorVelocity = std::abs(GetSetpoint().velocity.value() - GetGoal().velocity.value());

  if ((errorPosition <= kClimbPositionToleranceTurns) && (errorVelocity <= kClimbVelocityTolerancePerSecond)){
    return true;
  }
  else {
    return false;
  }
}

void ClimbSubsystem::MoveClimb(double speed) {
  // Inline construction of command goes here.
  m_climbMotor.Set(speed);
}

frc2::CommandPtr ClimbSubsystem::MoveUpCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_climbMotor.Set(-0.1); })
          .FinallyDo([this]{ m_climbMotor.Set(0); UpdateSetpoint(); });
}

frc2::CommandPtr ClimbSubsystem::MoveDownCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_climbMotor.Set(0.1); })
          .FinallyDo([this]{ m_climbMotor.Set(0); UpdateSetpoint(); });
}

frc2::CommandPtr ClimbSubsystem::MoveToAngleCommand(units::turn_t goal) {
  // Inline construction of command goes here. 
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return ReleaseClimbCommand()
  .AndThen([this, goal] {
            m_climbGoal = {goal, 0.0_tps }; //stop at goal
            m_climbSetpoint = m_trapezoidalProfile.Calculate(ClimbConstants::kDt, m_climbSetpoint, m_climbGoal);     
            SetpointControl();                     
  })
  .Until([this] {return IsGoalReached();})
  .FinallyDo([this] {
    ReleaseRatchet();
    UpdateSetpoint(); // update setpoint to current position and set velocity to 0 - then default command will keep this under control for us
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
  return RunOnce([this] {
    StopMotor();
    LockRatchet();
  });
}

frc2::CommandPtr ClimbSubsystem::ReleaseClimbCommand()
{
  return RunOnce([this] {
    ReleaseRatchet();
  });
}

//reset encoder
void ClimbSubsystem::ResetEncoder() {
  m_climbEncoder.SetPosition(ClimbConstants::resetEncoder.value());
}

void ClimbSubsystem::SetpointControl() {
  frc::SmartDashboard::PutNumber("positionSetpoint", m_climbSetpoint.position.value());
  frc::SmartDashboard::PutNumber("velocitySetpoint", m_climbSetpoint.velocity.value());
  frc::SmartDashboard::PutNumber("currentVelocity", m_climbEncoder.GetVelocity() / 60.0);
  m_closedLoopController.SetReference(m_climbSetpoint.position.value(),
                                      rev::spark::SparkLowLevel::ControlType::kPosition,
                                      rev::spark::kSlot0,
                                      m_climbFeedforward.Calculate(m_climbSetpoint.position, m_climbSetpoint.velocity).value());
}

frc2::CommandPtr ClimbSubsystem::SetpointControlCommand() {
  return Run([this] {
    SetpointControl();
  });
}

void ClimbSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  frc::SmartDashboard::PutNumber("climbEncoderValue", GetClimbAngle().value());  
}

void ClimbSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
