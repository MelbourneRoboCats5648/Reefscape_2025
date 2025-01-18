#include <numbers>

#include "subsystems/RightClimbSubsystem.h"

// possibly add smart dashboard from example for hard switches
#include <rev/config/SparkMaxConfig.h>
#include <frc2/command/Commands.h>

#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>




RightClimbSubsystem::RightClimbSubsystem() {
  // Implementation of subsystem constructor goes here.

  //PID Trapezoidal Controller
  //static constexpr units::second_t kDt = 20_ms; - fixme not in use

    // Note: These gains are fake, and will have to be tuned for your robot.- using different motor 
    //m_motor.SetPID(1.3, 0.0, 0.7);
  
  rev::spark::SparkMaxConfig motorConfig;
/*
   * Configure the encoder. For this specific example, we are using the
   * integrated encoder of the NEO, and we don't need to configure it. If
   * needed, we can adjust values like the position or velocity conversion
   * factors.
   */
  motorConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);

  /*
   * Configure the closed loop controller. We want to make sure we set the
   * feedback sensor as the primary encoder.
   */
  motorConfig.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .P(0.1)
      .I(0)
      .D(0)
      .OutputRange(-1, 1);
      /*
   * Apply the configuration to the SPARK MAX.
   *
   * kResetSafeParameters is used to get the SPARK MAX to a known state. This
   * is useful in case the SPARK MAX is replaced.
   *
   * kPersistParameters is used to ensure the configuration is not lost when
   * the SPARK MAX loses power. This is useful for power cycles that may occur
   * mid-operation.
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

 SparkRelativeEncoder m_encoder = m_motorController.GetEncoder(); 
  // Reset the position to 0 to start within the range of the soft limits
  m_encoder.SetPosition(0);


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

frc2::CommandPtr RightClimbSubsystem::RightClimbCommand(units::turn_t goal) {
  frc::TrapezoidProfile<units::turn_t>::State goalState = {goal, 0_tps};
  frc::TrapezoidProfile<units::turn_t>::State setpointState;
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this, &setpointState, goalState] {
      setpointState = m_profile.Calculate(RightClimbConstants::kDt, setpointState, goalState);

    // Send setpoint to offboard controller PID
      m_closedLoopController.SetReference(setpointState.position.value(),
                                        SparkMax::ControlType::kPosition);
      
    })
    .Until([this, goal]{
      auto error = goal - m_encoder.GetPosition();
    return units::math::abs(error)< RightClimbConstants::kGoalThreshold;
    });
      
}


void RightClimbSubsystem::Periodic() {

}
 
void RightClimbSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.

  
}

