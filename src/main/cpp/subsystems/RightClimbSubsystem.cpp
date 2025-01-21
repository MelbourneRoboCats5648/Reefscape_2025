#include "subsystems/RightClimbSubsystem.h"


RightClimbSubsystem::RightClimbSubsystem() {
  // Implementation of subsystem constructor goes here.
  rev::spark::SparkMaxConfig motorConfig;

// Set the idle mode to brake to stop immediately when reaching a limit
 motorConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake); 

/* TEMPORARILY REMOVED THE LIMIT SWITCH CODE BELOW TO TEST PID CONTROL FIRST */

// // Enable limit switches to stop the motor when they are closed
// //only hard switch code in this repo, will add others when organised
//motorConfig.limitSwitch
//     .ForwardLimitSwitchType(rev::spark::LimitSwitchConfig::Type::kNormallyOpen)
//     .ForwardLimitSwitchEnabled(true)
//     .ReverseLimitSwitchType(rev::spark::LimitSwitchConfig::Type::kNormallyOpen)
//     .ReverseLimitSwitchEnabled(true);

//  // Set the soft limits to stop the motor at -50 and 50 rotations
//  //will alter constants 
//  motorConfig.softLimit
//      .ForwardSoftLimit(50)
//      .ForwardSoftLimitEnabled(true)
//     .ReverseSoftLimit(-50)
//      .ReverseSoftLimitEnabled(true);

//PID Controller 
/*
* Configure the closed loop controller. We want to make sure we set the
* feedback sensor as the primary encoder.
*/
motorConfig.closedLoop
     .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
    .P(RightClimbConstants::kP)
    .I(RightClimbConstants::kI)
    .D(RightClimbConstants::kD)
    .OutputRange(-1, 1);
      /*
        /*
 * Apply the configuration to the SPARK MAX.
 *
// Reset the position to 0 to start within the range of the soft limits                  
m_encoder.SetPosition(0);

/*
 * Configure the encoder. For this specific example, we are using the
 * integrated encoder of the NEO, and we don't need to configure it. If
 * needed, we can adjust values like the position or velocity conversion
 * factors.
 */
  motorConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);

  
//Hard and Soft limit switch run parameters

      /*
   * Apply the configuration to the SPARK MAX.
   *
   * kResetSafeParameters is used to get the SPARK MAX to a known state. This
   * is useful in case the SPARK MAX is replaced.
   *
   * kPersistParameters is used to ensure the configuration is not lost when
   * the SPARK MAX loses power. This is useful for power cycles that may occur mid-operation.*/
   

  m_motor.Configure(motorConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                    rev::spark::SparkMax::PersistMode::kPersistParameters);

  // Reset the position to 0 to start within the range of the soft limits
  m_motor.GetEncoder().SetPosition(0);
}

frc2::CommandPtr RightClimbSubsystem::RightClimbUpCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_motorClimbRight.Set(rightClimbUpSpeed); })
          .FinallyDo([this]{m_motorClimbRight.Set(0);});
}

frc2::CommandPtr RightClimbSubsystem::RightClimbDownCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] {m_motorClimbRight.Set(rightClimbDownSpeed);})
          .FinallyDo([this]{m_motorClimbRight.Set(0);});
}

frc2::CommandPtr RightClimbSubsystem::RightClimb1Command(units::turn_t goal) {
  /*frc::TrapezoidProfile<units::turn_t>::State goalState = {goal, 0_tps};
  frc::TrapezoidProfile<units::turn_t>::State setpointState;
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem. */
  return Run([this] {
    m_rightClimbSetpoint = m_trapezoidalProfile.Calculate(RightClimbConstants::kDt, m_rightClimbSetpoint, m_rightClimbGoal);
    m_closedLoopController.SetReference(m_rightClimbSetpoint.position.value(), SparkLowLevel::ControlType::kPosition);
      })   
         .FinallyDo([this]{m_motorClimbRight.Set(0);});    
}
          // Send setpoint to offboard controller PID
      //m_closedLoopController.SetReference(setpointState.position.value(),  previously part of command
                                       // SparkMax::ControlType::kPosition); //- Closedloopcontroller will set setpoint values through motor controller


void RightClimbSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void RightClimbSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

