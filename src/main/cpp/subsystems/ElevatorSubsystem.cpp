#include "subsystems/ElevatorSubsystem.h"

#include <rev/SparkMax.h>

#include <rev/config/SparkMaxConfig.h>

ElevatorSubsystem::ElevatorSubsystem() {
  // Implementation of subsystem constructor goes here.
  rev::spark::SparkMaxConfig elevatorMotorConfig;

  /*
   * Set parameters that will apply to elevator motor.
   */
   elevatorMotorConfig.SmartCurrentLimit(50).SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake); 
   //fixme - test to find out the current limit

    // // Enable limit switches to stop the motor when they are closed
    // //only hard switch code in this repo, will add others when organised
    elevatorMotorConfig.limitSwitch
        .ForwardLimitSwitchType(rev::spark::LimitSwitchConfig::Type::kNormallyOpen)
        .ForwardLimitSwitchEnabled(true)
        .ReverseLimitSwitchType(rev::spark::LimitSwitchConfig::Type::kNormallyOpen)
        .ReverseLimitSwitchEnabled(true);

    //  // Set the soft limits to stop the motor at -50 and 50 rotations
    //  //will alter constants 
    elevatorMotorConfig.softLimit
      .ForwardSoftLimit(ElevatorConstants::extendSoftLimit.value())
      .ForwardSoftLimitEnabled(true)
      .ReverseSoftLimit(ElevatorConstants::retractSoftLimit.value())
      .ReverseSoftLimitEnabled(true);

  //PID Controller 
  /*
  * Configure the closed loop controller. We want to make sure we set the
  * feedback sensor as the primary encoder.
  */
  elevatorMotorConfig.closedLoop
     .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
    .P(ElevatorConstants::kP)
    .I(ElevatorConstants::kI)
    .D(ElevatorConstants::kD)
    .OutputRange(-1, 1);

  /*
  * Configure the encoder. For this specific example, we are using the
  * integrated encoder of the NEO, and we don't need to configure it. If
  * needed, we can adjust values like the position or velocity conversion
  * factors.
  */
  elevatorMotorConfig.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);

  //Hard and Soft limit switch run parameters
  /*
   * Apply the configuration to the SPARK MAX.
   *
   * kResetSafeParameters is used to get the SPARK MAX to a known state. This
   * is useful in case the SPARK MAX is replaced.
   *
   * kPersistParameters is used to ensure the configuration is not lost when
   * the SPARK MAX loses power. This is useful for power cycles that may occur mid-operation.*/
   

  m_motor.Configure(elevatorMotorConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                    rev::spark::SparkMax::PersistMode::kPersistParameters);

  // Reset the position to 0 to start within the range of the soft limits
  m_encoder.SetPosition(ElevatorConstants::resetEncoder.value());
  
}

frc2::CommandPtr ElevatorSubsystem::MoveUpCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_motor.Set(0.1);})
         .FinallyDo([this]{m_motor.Set(0);});
}

frc2::CommandPtr ElevatorSubsystem::MoveDownCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_motor.Set(-0.1);})
         .FinallyDo([this]{m_motor.Set(0);});
}

frc2::CommandPtr ElevatorSubsystem::MoveToLevelCommand(units::turn_t goal) {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem. */
  return Run([this, goal] {
            frc::TrapezoidProfile<units::turn>::State goalState = {goal, 0.0_tps }; //stop at goal
            m_elevatorSetpoint = m_trapezoidalProfile.Calculate(ElevatorConstants::kDt, m_elevatorSetpoint, goalState);
            m_closedLoopController.SetReference(m_elevatorSetpoint.position.value(), rev::spark::SparkLowLevel::ControlType::kPosition);
            })   
         .FinallyDo([this]{m_motor.Set(0);});
}

frc2::CommandPtr ElevatorSubsystem::StopAtLimitCommand() {
  // Inline construction of command goes here.
  return Run([this] {})
        .FinallyDo([this]{m_motor.Set(0);});
}

void ElevatorSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ElevatorSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
