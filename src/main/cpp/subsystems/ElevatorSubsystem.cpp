#include "subsystems/ElevatorSubsystem.h"
#include <rev/config/SparkMaxConfig.h>

using namespace ElevatorConstants;

ElevatorSubsystem::ElevatorSubsystem() {
  // Implementation of subsystem constructor goes here.
  rev::spark::SparkMaxConfig elevatorMotorSecondStageLeftConfig;
  rev::spark::SparkMaxConfig elevatorMotorSecondStageRightConfig;
  rev::spark::SparkMaxConfig elevatorMotorThirdStageConfig;
  /*
   * Set parameters that will apply to elevator motor.
   */
    elevatorMotorSecondStageLeftConfig.SmartCurrentLimit(ElevatorConstants::kCurrentLimit).SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    elevatorMotorSecondStageRightConfig.SmartCurrentLimit(ElevatorConstants::kCurrentLimit).SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    elevatorMotorThirdStageConfig.SmartCurrentLimit(ElevatorConstants::kCurrentLimit).SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
   //fixme - test to find out the current limit

  //Second Stage Limits
    // // Enable limit switches to stop the motor when they are closed
    elevatorMotorSecondStageLeftConfig.limitSwitch
      .ReverseLimitSwitchType(rev::spark::LimitSwitchConfig::Type::kNormallyOpen)
      .ReverseLimitSwitchEnabled(true);

    elevatorMotorSecondStageLeftConfig.softLimit
      .ForwardSoftLimit(ElevatorConstants::extendSoftLimitSecondStage.value())
      .ForwardSoftLimitEnabled(true);

  //Third Stage Limits
    // // Enable limit switches to stop the motor when they are closed
    // //only hard switch code in this repo, will add others when organised
    elevatorMotorThirdStageConfig.limitSwitch
      .ReverseLimitSwitchType(rev::spark::LimitSwitchConfig::Type::kNormallyOpen)
      .ReverseLimitSwitchEnabled(true);

    //  // Set the soft limits to stop the motor at -50 and 50 rotations
    //  //will alter constants 
    elevatorMotorThirdStageConfig.softLimit
      .ForwardSoftLimit(ElevatorConstants::extendSoftLimitThirdStage.value())
      .ForwardSoftLimitEnabled(true);

  //PID Controller 
  /*
  * Configure the closed loop controller. We want to make sure we set the
  * feedback sensor as the primary encoder.
  */
  elevatorMotorSecondStageLeftConfig.closedLoop
     .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
    .P(ElevatorConstants::kP)
    .I(ElevatorConstants::kI)
    .D(ElevatorConstants::kD)
    .OutputRange(-ElevatorConstants::maxOutput, ElevatorConstants::maxOutput);


//PID Controller Third Stage
  /*
  * Configure the closed loop controller. We want to make sure we set the
  * feedback sensor as the primary encoder.
  */
  elevatorMotorThirdStageConfig.closedLoop
     .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
    .P(ElevatorConstants::kP)
    .I(ElevatorConstants::kI)
    .D(ElevatorConstants::kD)
    .OutputRange(-ElevatorConstants::maxOutput, ElevatorConstants::maxOutput);

  /*
  * Configure the encoder. For this specific example, we are using the
  * integrated encoder of the NEO, and we don't need to configure it. If
  * needed, we can adjust values like the position or velocity conversion
  * factors.
  */

  elevatorMotorSecondStageLeftConfig.encoder.PositionConversionFactor(ElevatorConstants::gearRatio).VelocityConversionFactor(ElevatorConstants::gearRatio);
  elevatorMotorSecondStageRightConfig.encoder.PositionConversionFactor(ElevatorConstants::gearRatio).VelocityConversionFactor(ElevatorConstants::gearRatio);
  elevatorMotorThirdStageConfig.encoder.PositionConversionFactor(ElevatorConstants::gearRatio).VelocityConversionFactor(ElevatorConstants::gearRatio);
  //Hard and Soft limit switch run parameters
// right motor will follow the inverted output of left motor to drive shaft
  bool invertOutput = true;
  elevatorMotorSecondStageRightConfig.Follow(m_motorSecondStageLeft, invertOutput);


  /*
   * Apply the configuration to the SPARK MAX.
   *
   * kResetSafeParameters is used to get the SPARK MAX to a known state. This
   * is useful in case the SPARK MAX is replaced.
   *
   * kPersistParameters is used to ensure the configuration is not lost when
   * the SPARK MAX loses power. This is useful for power cycles that may occur mid-operation.*/
   
  m_motorSecondStageLeft.Configure(elevatorMotorSecondStageLeftConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                    rev::spark::SparkMax::PersistMode::kPersistParameters);
  m_motorSecondStageRight.Configure(elevatorMotorSecondStageRightConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                    rev::spark::SparkMax::PersistMode::kPersistParameters);
  m_motorThirdStage.Configure(elevatorMotorThirdStageConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                    rev::spark::SparkMax::PersistMode::kPersistParameters);

  // Reset the position to 0 to start within the range of the soft limits
  m_encoderLeft.SetPosition(ElevatorConstants::resetEncoder.value());
  m_encoderRight.SetPosition(ElevatorConstants::resetEncoder.value());
  m_encoderThirdStage.SetPosition(ElevatorConstants::resetEncoder.value());

}

units::meter_t ElevatorSubsystem::GetElevatorHeight() {
  //using left encoder as position reference
  return m_encoderLeft.GetPosition() * ElevatorConstants::distancePerTurn;
         m_encoderRight.GetPosition() * ElevatorConstants::distancePerTurn;
         m_encoderThirdStage.GetPosition() * ElevatorConstants::distancePerTurn;
}

void ElevatorSubsystem::UpdateSetpoint() {  
  m_elevatorSetpoint.position = GetElevatorHeight();
  m_elevatorSetpoint.velocity = 0.0_mps; 
}

frc::TrapezoidProfile<units::meter>::State& ElevatorSubsystem::GetSetpoint() {
  return  m_elevatorSetpoint;
} 

frc::TrapezoidProfile<units::meter>::State& ElevatorSubsystem::GetGoal() {
  return  m_elevatorGoal;
} 

bool ElevatorSubsystem::IsGoalReached() {
  double errorPosition = std::abs(GetSetpoint().position.value() - GetGoal().position.value());
  double errorVelocity = std::abs(GetSetpoint().velocity.value() - GetGoal().velocity.value());

  if((errorPosition <= kElevatorPositionToleranceMetres) && (errorVelocity <= kElevatorVelocityTolerancePerSecond)){
    return true;
  }
  else {
    return false;
  }

}

void ElevatorSubsystem::ResetMotor() {  
  m_motorSecondStageLeft.Set(0);
}

void ElevatorSubsystem::ResetEncoder() {
  m_encoderLeft.SetPosition(0.0);
  m_encoderRight.SetPosition(0.0);
}

frc2::CommandPtr ElevatorSubsystem::MoveUpCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_motorSecondStageLeft.Set(0.1);})
         .FinallyDo([this]{
          ResetMotor();
          UpdateSetpoint();
         });
}

frc2::CommandPtr ElevatorSubsystem::MoveDownCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_motorSecondStageLeft.Set(-0.1);})
         .FinallyDo([this]{
          ResetMotor();
          UpdateSetpoint();
         });
}

frc2::CommandPtr ElevatorSubsystem::MoveToHeightCommand(units::meter_t heightGoal) {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem. */
  if(heightGoal <= ElevatorConstants::kMaxSecondStageHeight){
    return (MoveSecondStageToHeightCommand(heightGoal))
    .AlongWith(MoveThirdStageToHeightCommand(0_m));
  }
  else{
    return (MoveSecondStageToHeightCommand(ElevatorConstants::kMaxSecondStageHeight))
    .AlongWith(MoveThirdStageToHeightCommand(heightGoal - ElevatorConstants::kMaxSecondStageHeight));
  }
}

frc2::CommandPtr ElevatorSubsystem::MoveSecondStageToHeightCommand(units::meter_t goal) {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem. */
  return Run([this, goal] {
            m_elevatorGoal = {goal, 0.0_mps }; //stop at goal
            m_elevatorSetpoint = m_trapezoidalProfile.Calculate(ElevatorConstants::kDt, m_elevatorSetpoint, m_elevatorGoal);
            frc::SmartDashboard::PutNumber("trapazoidalSecondStageSetpoint", m_elevatorSetpoint.position.value());
            m_closedLoopControllerLeft.SetReference(m_elevatorGoal.position.value(), 
                                                rev::spark::SparkLowLevel::ControlType::kPosition,
                                                rev::spark::kSlot0,
                                                m_elevatorFeedforward.Calculate(m_elevatorSetpoint.velocity).value());
            
            })   
        .FinallyDo([this]{m_motorSecondStageLeft.Set(0);});
}

frc2::CommandPtr ElevatorSubsystem::MoveThirdStageToHeightCommand(units::meter_t goal) {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem. */
  return Run([this, goal] {
            m_elevatorGoal = {goal, 0.0_mps }; //stop at goal
            m_elevatorSetpoint = m_trapezoidalProfile.Calculate(ElevatorConstants::kDt, m_elevatorSetpoint, m_elevatorGoal);
            frc::SmartDashboard::PutNumber("trapazoidalThirdStageSetpoint", m_elevatorSetpoint.position.value());
            m_closedLoopControllerThirdStage.SetReference(m_elevatorGoal.position.value(), 
                                                rev::spark::SparkLowLevel::ControlType::kPosition,
                                                rev::spark::kSlot0,
                                                m_elevatorFeedforward.Calculate(m_elevatorSetpoint.velocity).value());
            })   
        .FinallyDo([this]{m_motorThirdStage.Set(0);});
}

//To move down supply a negative
frc2::CommandPtr ElevatorSubsystem::MoveUpBy(units::meter_t height) {
      units::meter_t moveGoal = (GetElevatorHeight() + height);
      return MoveToHeightCommand(moveGoal);
}

void ElevatorSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  frc::SmartDashboard::PutNumber("encoderSecondStageLeftValue", m_encoderLeft.GetPosition());
  frc::SmartDashboard::PutNumber("encoderSecondStageRightValue", m_encoderRight.GetPosition());
  frc::SmartDashboard::PutNumber("encoderThirdStageValue", m_encoderThirdStage.GetPosition());
}

void ElevatorSubsystem::OnLimitSwitchActivation() {
      if(m_limitSwitchElevator.Get()) {
      ResetMotor();
      ResetEncoder();
    }
}

void ElevatorSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
