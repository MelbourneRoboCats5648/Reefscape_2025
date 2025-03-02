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
  ResetEncoder();

  SetDefaultCommand(SetpointControlCommand());
}

units::meter_t ElevatorSubsystem::GetElevatorFirstStageHeight() {
  //using left encoder as position reference
  return m_encoderLeft.GetPosition() * ElevatorConstants::distancePerTurnFirstStage;
}

units::meter_t ElevatorSubsystem::GetElevatorSecondStageHeight() {
  //using left encoder as position reference
  return m_encoderSecondStage.GetPosition() * ElevatorConstants::distancePerTurnSecondStage;
}

units::meter_t ElevatorSubsystem::GetElevatorHeight() {
  return GetElevatorFirstStageHeight() + GetElevatorSecondStageHeight();
}

void ElevatorSubsystem::UpdateFirstStageSetpoint() {  
  m_elevatorFirstStageSetpoint.position = GetElevatorFirstStageHeight();
  m_elevatorFirstStageSetpoint.velocity = 0.0_mps;
}

void ElevatorSubsystem::UpdateSecondStageSetpoint() {  
  m_elevatorFirstStageSetpoint.position = GetElevatorSecondStageHeight();
  m_elevatorFirstStageSetpoint.velocity = 0.0_mps;
}

frc::TrapezoidProfile<units::meter>::State& ElevatorSubsystem::GetFirstStageSetpoint() {
  return  m_elevatorFirstStageSetpoint;
} 

frc::TrapezoidProfile<units::meter>::State& ElevatorSubsystem::GetFirstStageGoal() {
  return  m_elevatorFirstStageGoal;
}

bool ElevatorSubsystem::IsFirstStageGoalReached() {
  double errorPosition = std::abs(GetFirstStageSetpoint().position.value() - GetFirstStageGoal().position.value());
  double errorVelocity = std::abs(GetFirstStageSetpoint().velocity.value() - GetFirstStageGoal().velocity.value());

  if((errorPosition <= kElevatorPositionToleranceMetres) && (errorVelocity <= kElevatorVelocityTolerancePerSecond)) {
    return true;
  }
  else {
    return false;
  }
}

frc::TrapezoidProfile<units::meter>::State& ElevatorSubsystem::GetSecondStageSetpoint() {
  return  m_elevatorSecondStageSetpoint;
} 

frc::TrapezoidProfile<units::meter>::State& ElevatorSubsystem::GetSecondStageGoal() {
  return  m_elevatorSecondStageGoal;
} 

bool ElevatorSubsystem::IsSecondStageGoalReached() {
  double errorPosition = std::abs(GetSecondStageSetpoint().position.value() - GetSecondStageGoal().position.value());
  double errorVelocity = std::abs(GetSecondStageSetpoint().velocity.value() - GetSecondStageGoal().velocity.value());

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
  double encoderValue = ElevatorConstants::resetEncoder.value();
  m_encoderLeft.SetPosition(encoderValue);
  m_encoderRight.SetPosition(encoderValue);
  m_encoderSecondStage.SetPosition(encoderValue);
}

void ElevatorSubsystem::FirstStageSetpointControl() {
  frc::SmartDashboard::PutNumber("ElevatorFirstStage/positionSetpoint", m_elevatorFirstStageSetpoint.position.value());
  frc::SmartDashboard::PutNumber("ElevatorFirstStage/velocitySetpoint", m_elevatorFirstStageSetpoint.velocity.value());
  frc::SmartDashboard::PutNumber("ElevatorFirstStage/currentVelocity", m_encoderLeft.GetVelocity() / 60.0);
  m_closedLoopControllerLeft.SetReference(m_elevatorFirstStageSetpoint.position.value(), 
                                      rev::spark::SparkLowLevel::ControlType::kPosition,
                                      rev::spark::kSlot0,
                                      m_elevatorFeedforward.Calculate(m_elevatorFirstStageSetpoint.velocity).value());
}

void ElevatorSubsystem::SecondStageSetpointControl() {
  frc::SmartDashboard::PutNumber("ElevatorSecondStage/positionSetpoint", m_elevatorSecondStageSetpoint.position.value());
  frc::SmartDashboard::PutNumber("ElevatorSecondStage/velocitySetpoint", m_elevatorSecondStageSetpoint.velocity.value());
  frc::SmartDashboard::PutNumber("ElevatorSecondStage/currentVelocity", m_encoderSecondStage.GetVelocity() / 60.0);
  m_closedLoopControllerSecondStage.SetReference(m_elevatorSecondStageSetpoint.position.value(), 
                                      rev::spark::SparkLowLevel::ControlType::kPosition,
                                      rev::spark::kSlot0,
                                      m_elevatorFeedforward.Calculate(m_elevatorSecondStageSetpoint.velocity).value());
}

frc2::CommandPtr ElevatorSubsystem::SetpointControlCommand() {
  return Run([this] {
    FirstStageSetpointControl();
    SecondStageSetpointControl();
  });
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
          UpdateFirstStageSetpoint();
         });
}

frc2::CommandPtr ElevatorSubsystem::MoveDownCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_motorFirstStageLeft.Set(-0.1); })
      .FinallyDo([this] {
          ResetMotor();
          UpdateFirstStageSetpoint();
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
    .AlongWith(MoveFirstStageToHeightCommand(heightGoal - ElevatorConstants::kMaxSecondStageHeight));
  }
}

frc2::CommandPtr ElevatorSubsystem::MoveFirstStageToHeightCommand(units::meter_t goal) {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem. */
  return Run([this, goal] {
            m_elevatorFirstStageGoal = {goal, 0.0_mps }; //stop at goal
            m_elevatorFirstStageSetpoint = m_trapezoidalProfile.Calculate(ElevatorConstants::kDt, m_elevatorFirstStageSetpoint, m_elevatorFirstStageGoal);
            FirstStageSetpointControl();
        })
        .Until([this] {return IsFirstStageGoalReached();})
        .FinallyDo([this] {UpdateFirstStageSetpoint(); });
}

frc2::CommandPtr ElevatorSubsystem::MoveSecondStageToHeightCommand(units::meter_t goal) {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem. */
  return Run([this, goal] {
            m_elevatorSecondStageGoal = {goal, 0.0_mps }; //stop at goal
            m_elevatorSecondStageSetpoint = m_trapezoidalProfile.Calculate(ElevatorConstants::kDt, m_elevatorSecondStageSetpoint, m_elevatorSecondStageGoal);
            SecondStageSetpointControl();
        })
        .Until([this] {return IsSecondStageGoalReached();})
        .FinallyDo([this] {UpdateSecondStageSetpoint(); });
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
