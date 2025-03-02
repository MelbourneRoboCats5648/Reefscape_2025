#include "subsystems/ElevatorSubsystem.h"
#include <rev/config/SparkMaxConfig.h>

using namespace ElevatorConstants;

ElevatorSubsystem::ElevatorSubsystem() {
  // Implementation of subsystem constructor goes here.
  rev::spark::SparkMaxConfig elevatorMotorFirstStageLeftConfig;
  rev::spark::SparkMaxConfig elevatorMotorFirstStageRightConfig;
  rev::spark::SparkMaxConfig elevatorMotorSecondStageConfig;

   //Set parameters that will apply to elevator motor.
    elevatorMotorFirstStageLeftConfig.SmartCurrentLimit(ElevatorConstants::kCurrentLimit).SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    elevatorMotorFirstStageRightConfig.SmartCurrentLimit(ElevatorConstants::kCurrentLimit).SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    elevatorMotorSecondStageConfig.SmartCurrentLimit(ElevatorConstants::kCurrentLimit).SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

  // First Stage Limits
    // Enable limit switches to stop the motor when they are closed
    elevatorMotorFirstStageLeftConfig.limitSwitch
      .ReverseLimitSwitchType(rev::spark::LimitSwitchConfig::Type::kNormallyOpen)
      .ReverseLimitSwitchEnabled(true);

    elevatorMotorFirstStageLeftConfig.softLimit
      .ForwardSoftLimit(ElevatorConstants::extendSoftLimitFirstStage.value())
      .ForwardSoftLimitEnabled(true);

  //Second Stage Limits
    // Enable limit switches to stop the motor when they are closed
    elevatorMotorSecondStageConfig.limitSwitch
      .ReverseLimitSwitchType(rev::spark::LimitSwitchConfig::Type::kNormallyOpen)
      .ReverseLimitSwitchEnabled(true);

    // Set the soft limits to stop the motor at -50 and 50 rotations
    // will alter constants
    elevatorMotorSecondStageConfig.softLimit
      .ForwardSoftLimit(ElevatorConstants::extendSoftLimitSecondStage.value())
      .ForwardSoftLimitEnabled(true);

  //PID Controller 
  /* Configure the closed loop controller. We want to make sure we set the
  * feedback sensor as the primary encoder. */
  elevatorMotorFirstStageLeftConfig.closedLoop
     .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
    .P(ElevatorConstants::kP)
    .I(ElevatorConstants::kI)
    .D(ElevatorConstants::kD)
    .OutputRange(-ElevatorConstants::maxOutput, ElevatorConstants::maxOutput);

  //PID Controller Second Stage
  /* Configure the closed loop controller. We want to make sure we set the
  * feedback sensor as the primary encoder. */
  elevatorMotorSecondStageConfig.closedLoop
     .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      /* Set PID values for position control. We don't need to pass a closed
      * loop slot, as it will default to slot 0. */
    .P(ElevatorConstants::kP)
    .I(ElevatorConstants::kI)
    .D(ElevatorConstants::kD)
    .OutputRange(-ElevatorConstants::maxOutput, ElevatorConstants::maxOutput);

  // Configure the encoder.
  elevatorMotorFirstStageLeftConfig.encoder.PositionConversionFactor(ElevatorConstants::gearRatio).VelocityConversionFactor(ElevatorConstants::gearRatio);
  elevatorMotorFirstStageRightConfig.encoder.PositionConversionFactor(ElevatorConstants::gearRatio).VelocityConversionFactor(ElevatorConstants::gearRatio);
  elevatorMotorSecondStageConfig.encoder.PositionConversionFactor(ElevatorConstants::gearRatio).VelocityConversionFactor(ElevatorConstants::gearRatio);
  
  //Hard and Soft limit switch run parameters
  // right motor will follow the inverted output of left motor to drive shaft
  bool invertOutput = true;
  elevatorMotorFirstStageRightConfig.Follow(m_motorFirstStageLeft, invertOutput);

  /* Apply the configuration to the SPARK MAX.
   *
   * kResetSafeParameters is used to get the SPARK MAX to a known state. 
   * This is useful in case the SPARK MAX is replaced.
   *
   * kPersistParameters is used to ensure the configuration is not lost when the SPARK MAX loses power. 
   * This is useful for power cycles that may occur mid-operation.*/
  
  m_motorFirstStageLeft.Configure(elevatorMotorFirstStageLeftConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                    rev::spark::SparkMax::PersistMode::kPersistParameters);
  m_motorFirstStageRight.Configure(elevatorMotorFirstStageRightConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                    rev::spark::SparkMax::PersistMode::kPersistParameters);
  m_motorSecondStage.Configure(elevatorMotorSecondStageConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                    rev::spark::SparkMax::PersistMode::kPersistParameters);

  // Reset the position to 0 to start within the range of the soft limits
  m_encoderLeft.SetPosition(ElevatorConstants::resetEncoder.value());
  m_encoderRight.SetPosition(ElevatorConstants::resetEncoder.value());
  m_encoderSecondStage.SetPosition(ElevatorConstants::resetEncoder.value());

}

units::meter_t ElevatorSubsystem::GetElevatorHeight() {
  //using left encoder as position reference
  return m_encoderLeft.GetPosition() * ElevatorConstants::distancePerTurnFirstStage;
         m_encoderRight.GetPosition() * ElevatorConstants::distancePerTurnFirstStage;
         m_encoderSecondStage.GetPosition() * ElevatorConstants::distancePerTurnSecondStage;
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

  if((errorPosition <= kElevatorPositionToleranceMetres) && (errorVelocity <= kElevatorVelocityTolerancePerSecond)) {
    return true;
  }
  else {
    return false;
  }
}

void ElevatorSubsystem::ResetMotor() {  
  m_motorFirstStageLeft.Set(0);
}

void ElevatorSubsystem::ResetEncoder() {
  m_encoderLeft.SetPosition(0.0);
  m_encoderRight.SetPosition(0.0);
}

void ElevatorSubsystem::MoveSecondStage(double speed) {
  // Inline construction of command goes here.
  m_motorSecondStage.Set(speed);
}

frc2::CommandPtr ElevatorSubsystem::MoveUpCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_motorFirstStageLeft.Set(0.1); })
      .FinallyDo([this] {
          ResetMotor();
          UpdateSetpoint();
         });
}

frc2::CommandPtr ElevatorSubsystem::MoveDownCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_motorFirstStageLeft.Set(-0.1); })
      .FinallyDo([this] {
          ResetMotor();
          UpdateSetpoint();
         });
}

frc2::CommandPtr ElevatorSubsystem::MoveToHeightCommand(units::meter_t heightGoal) {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem. */
  if(heightGoal <= ElevatorConstants::kMaxSecondStageHeight) {
    return (MoveSecondStageToHeightCommand(heightGoal))
    .AlongWith(MoveFirstStageToHeightCommand(0_m));
  }
  else {
    return (MoveSecondStageToHeightCommand(ElevatorConstants::kMaxSecondStageHeight))
    .AndThen(MoveFirstStageToHeightCommand(heightGoal - ElevatorConstants::kMaxSecondStageHeight));
  }
}

frc2::CommandPtr ElevatorSubsystem::MoveFirstStageToHeightCommand(units::meter_t goal) {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem. */
  return Run([this, goal] {
            m_elevatorGoal = {goal, 0.0_mps }; //stop at goal
            m_elevatorSetpoint = m_trapezoidalProfile.Calculate(ElevatorConstants::kDt, m_elevatorSetpoint, m_elevatorGoal);
            frc::SmartDashboard::PutNumber("trapazoidalFirstStageSetpoint", m_elevatorSetpoint.position.value());
            m_closedLoopControllerLeft.SetReference(m_elevatorGoal.position.value(), 
                                                rev::spark::SparkLowLevel::ControlType::kPosition,
                                                rev::spark::kSlot0,
                                                m_elevatorFeedforward.Calculate(m_elevatorSetpoint.velocity).value());
            
            })   
        .FinallyDo([this] {m_motorFirstStageLeft.Set(0); });
}

frc2::CommandPtr ElevatorSubsystem::MoveSecondStageToHeightCommand(units::meter_t goal) {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem. */
  return Run([this, goal] {
            m_elevatorGoal = {goal, 0.0_mps }; //stop at goal
            m_elevatorSetpoint = m_trapezoidalProfile.Calculate(ElevatorConstants::kDt, m_elevatorSetpoint, m_elevatorGoal);
            frc::SmartDashboard::PutNumber("trapazoidalSecondStageSetpoint", m_elevatorSetpoint.position.value());
            m_closedLoopControllerSecondStage.SetReference(m_elevatorGoal.position.value(), 
                                                rev::spark::SparkLowLevel::ControlType::kPosition,
                                                rev::spark::kSlot0,
                                                m_elevatorFeedforward.Calculate(m_elevatorSetpoint.velocity).value());
            })   
        .FinallyDo([this]{m_motorSecondStage.Set(0);});
}

//To move down supply a negative
frc2::CommandPtr ElevatorSubsystem::MoveUpBy(units::meter_t height) {
      units::meter_t moveGoal = (GetElevatorHeight() + height);
      return MoveToHeightCommand(moveGoal);
}

void ElevatorSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  frc::SmartDashboard::PutNumber("encoderFirstStageLeftValue", m_encoderLeft.GetPosition());
  frc::SmartDashboard::PutNumber("encoderFirstStageRightValue", m_encoderRight.GetPosition());
  frc::SmartDashboard::PutNumber("encoderSecondStageValue", m_encoderSecondStage.GetPosition());
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
