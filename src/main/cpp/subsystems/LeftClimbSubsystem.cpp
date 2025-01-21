#include "subsystems/LeftClimbSubsystem.h"

LeftClimbSubsystem::LeftClimbSubsystem() {
  // Implementation of subsystem constructor goes here.
  rev::spark::SparkMaxConfig motorConfig;
  //configuring the encoder:
  motorConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
  //Configure the closed loop controller. We want to make sure we set the feedback sensor as the primary encoder.
  motorConfig.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed loop slot, as it will default to slot 0.
      .P(LeftClimbConstants::kP)
      .I(LeftClimbConstants::kI)
      .D(LeftClimbConstants::kD)
      .OutputRange(-1, 1);

// Set the idle mode to brake to stop immediately when reaching a limit
 motorConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

/* TEMPORARILY REMOVED THE LIMIT SWITCH CODE BELOW TO TEST PID CONTROL FIRST */

// // Enable limit switches to stop the motor when they are closed
// //only hard switch code in this repo, will add others when organised
//  motorConfig.limitSwitch
//       .ForwardLimitSwitchType(rev::spark::LimitSwitchConfig::Type::kNormallyOpen)
//       .ForwardLimitSwitchEnabled(true)
//       .ReverseLimitSwitchType(rev::spark::LimitSwitchConfig::Type::kNormallyOpen)
//       .ReverseLimitSwitchEnabled(true);

//   // Set the soft limits to stop the motor at set number of rotations
//   //will alter constants 
//   motorConfig.softLimit
//       .ForwardSoftLimit(extendSoftLimit)
//       .ForwardSoftLimitEnabled(true)
//       .ReverseSoftLimit(retractSoftLimit)
//       .ReverseSoftLimitEnabled(true);


   /* Apply the configuration to the SPARK MAX.
   * kResetSafeParameters is used to get the SPARK MAX to a known state. This is useful in case the SPARK MAX is replaced. 
   * kPersistParameters is used to ensure the configuration is not lost when the SPARK MAX loses power. This is useful for power cycles that may occur mid-operation.*/
  m_motorController.Configure(motorConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                              rev::spark::SparkMax::PersistMode::kPersistParameters);
  
  // Reset the position to 0 to start within the range of the soft limits
  m_motorController.GetEncoder().SetPosition(0);
}

frc2::CommandPtr LeftClimbSubsystem::LeftClimbUpCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_motorClimbLeft.Set(leftClimbUpSpeed);})
          .FinallyDo([this]{m_motorClimbLeft.Set(0);});
}

frc2::CommandPtr LeftClimbSubsystem::LeftClimbDownCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_motorClimbLeft.Set(leftClimbDownSpeed);})
          .FinallyDo([this]{m_motorClimbLeft.Set(0);});
}

frc2::CommandPtr LeftClimbSubsystem::LeftClimbL1Command(units::turn_t goal) {
  return Run([this] {
          m_leftClimbSetpoint = m_TrapezoidalProfile.Calculate(LeftClimbConstants::kDt, m_leftClimbSetpoint, m_leftClimbGoal);
          m_closedLoopController.SetReference(m_leftClimbSetpoint.position.value(), SparkLowLevel::ControlType::kPosition);
        })
        .FinallyDo([this]{m_motorClimbLeft.Set(0);});
}

void LeftClimbSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void LeftClimbSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
