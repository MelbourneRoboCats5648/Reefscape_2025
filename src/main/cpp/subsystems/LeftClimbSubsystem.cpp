#include "subsystems/LeftClimbSubsystem.h"

LeftClimbSubsystem::LeftClimbSubsystem() {
  // Implementation of subsystem constructor goes here.
  rev::spark::SparkMaxConfig motorConfig;
  //configuring the encoder:
  motorConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
  //Configure the closed loop controller. We want to make sure we set the feedback sensor as the primary encoder.
  motorConfig.closedLoop
      .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed loop slot, as it will default to slot 0.
      .P(LeftClimbConstants::kP)
      .I(LeftClimbConstants::kI)
      .D(LeftClimbConstants::kD)
      .OutputRange(-1, 1);

 // Set the idle mode to brake to stop immediately when reaching a limit
 motorConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

   /* Apply the configuration to the SPARK MAX.
   * kResetSafeParameters is used to get the SPARK MAX to a known state. This is useful in case the SPARK MAX is replaced. 
   * kPersistParameters is used to ensure the configuration is not lost when the SPARK MAX loses power. This is useful for power cycles that may occur mid-operation.*/
  m_motorController.Configure(motorConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                              rev::spark::SparkMax::PersistMode::kPersistParameters);
  
  // Reset the position to 0 to start within the range of the soft limits
  m_encoder.SetPosition(0);
}

void LeftClimbSubsystem::RobotReset() {
  m_encoder.SetPosition(0);
  m_motorController.Set(0);
}

frc2::CommandPtr LeftClimbSubsystem::LeftClimbUpCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_motorController.Set(LeftClimbConstants::leftClimbUpSpeed);})
          .FinallyDo([this]{RobotReset();});
}

frc2::CommandPtr LeftClimbSubsystem::LeftClimbDownCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_motorController.Set(LeftClimbConstants::leftClimbDownSpeed);})
          .FinallyDo([this]{RobotReset();});
}

frc2::CommandPtr LeftClimbSubsystem::LeftClimbDefaultCommand(units::turn_t goal) {
  return Run([this, goal] {
          frc::TrapezoidProfile<units::turn>::State goalState = { goal, 0.0_tps }; // stop at goal
          m_leftClimbSetpoint = m_TrapezoidalProfile.Calculate(LeftClimbConstants::kDt, m_leftClimbSetpoint, goalState);
          m_closedLoopController.SetReference(m_leftClimbSetpoint.position.value(), rev::spark::SparkLowLevel::ControlType::kPosition);
        })
        .FinallyDo([this]{RobotReset();});
}

frc2::CommandPtr LeftClimbSubsystem::MoveToClimbLevel(ClimbState climbState) { 
  units::turn_t climbGoal;
  switch(climbState) {
    case (ClimbState::INITIAL): {
      climbGoal = GoalConstants::m_climbGoalL1;
      break;
    }
    case (ClimbState::EXTENDED): {
      climbGoal = GoalConstants::m_climbGoalRetract;
      break;
    }
    default: {
      climbGoal = GoalConstants::m_climbGoalRetract;    }
  }
}

void LeftClimbSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void LeftClimbSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
