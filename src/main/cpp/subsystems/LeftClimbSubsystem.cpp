#include "subsystems/LeftClimbSubsystem.h"

LeftClimbSubsystem::LeftClimbSubsystem() {
  // Implementation of subsystem constructor goes here.

  rev::spark::SparkMaxConfig motorConfig;
  //configuring the encoder:
  motorConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
  /*
   * Configure the closed loop controller. We want to make sure we set the feedback sensor as the primary encoder.
   */
  motorConfig.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed loop slot, as it will default to slot 0.
      .P(0.1)
      .I(0)
      .D(0)
      .OutputRange(-1, 1);
   /*
   * kResetSafeParameters is used to get the SPARK MAX to a known state. This is useful in case the SPARK MAX is replaced.
   * kPersistParameters is used to ensure the configuration is not lost when the SPARK MAX loses power. This is useful for power cycles that may occur mid-operation.
   */
  m_motorController.Configure(motorConfig, SparkBase::ResetMode::kResetSafeParameters,
                    SparkBase::PersistMode::kPersistParameters);
// Set the idle mode to brake to stop immediately when reaching a limit
 motorConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake); 

// Enable limit switches to stop the motor when they are closed
//only hard switch code in this repo, will add others when organised
 motorConfig.limitSwitch
      .ForwardLimitSwitchType(rev::spark::LimitSwitchConfig::Type::kNormallyOpen)
      .ForwardLimitSwitchEnabled(true)
      .ReverseLimitSwitchType(rev::spark::LimitSwitchConfig::Type::kNormallyOpen)
      .ReverseLimitSwitchEnabled(true);

  // Set the soft limits to stop the motor at -50 and 50 rotations
  //will alter constants 
  motorConfig.softLimit
      .ForwardSoftLimit(50)
      .ForwardSoftLimitEnabled(true)
      .ReverseSoftLimit(-50)
      .ReverseSoftLimitEnabled(true);
      /*
   * Apply the configuration to the SPARK MAX.
   *
   * kResetSafeParameters is used to get the SPARK MAX to a known state. This
   * is useful in case the SPARK MAX is replaced.
   *
   * kPersistParameters is used to ensure the configuration is not lost when
   * the SPARK MAX loses power. This is useful for power cycles that may occur mid-operation.*/
   
  m_motorController.Configure(motorConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                    rev::spark::SparkMax::PersistMode::kNoPersistParameters);
//Encoder:
  SparkRelativeEncoder m_encoder = m_motorController.GetEncoder(); 
  // Reset the position to 0 to start within the range of the soft limits
    m_encoder.SetPosition(0);

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

frc2::CommandPtr LeftClimbSubsystem::LeftClimbCommand(units::turn_t goal) {
  //motion profiling stuff
    frc::TrapezoidProfile<units::turn_t>::State goalState = {goal, 0_tps};
    frc::TrapezoidProfile<units::turn_t>::State setpointState;
  return Run([this, &setpointState, goalState] {
      setpointState = m_profile.Calculate(LeftClimbConstants::kDt, setpointState, goalState);

    // Send setpoint to offboard controller PID
      m_closedLoopController.SetReference(setpointState.position.value(),
                                        SparkMax::ControlType::kPosition);
      
    })
    .Until([this, goal]{
      auto error = goal - m_encoder.GetPosition();
    return units::math::abs(error)< LeftClimbConstants::kGoalThreshold;
    });
}

void LeftClimbSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void LeftClimbSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
