#include "subsystems/ElevatorSubsystem.h"
#include <rev/config/SparkMaxConfig.h>

using namespace ElevatorConstants;

ElevatorSubsystem::ElevatorSubsystem() {
  // Implementation of subsystem constructor goes here.
  rev::spark::SparkMaxConfig elevatorMotorLeftConfig;
  rev::spark::SparkMaxConfig elevatorMotorRightConfig;

  /*
   * Set parameters that will apply to elevator motor.
   */
  elevatorMotorLeftConfig.SmartCurrentLimit(ElevatorConstants::kCurrentLimit).SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
  elevatorMotorRightConfig.SmartCurrentLimit(ElevatorConstants::kCurrentLimit).SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
   //fixme - test to find out the current limit

  //Hard and Soft limit switch run parameters
    // // Enable limit switches to stop the motor when they are closed
    // //only hard switch code in this repo, will add others when organised
    elevatorMotorLeftConfig.limitSwitch
        .ReverseLimitSwitchType(rev::spark::LimitSwitchConfig::Type::kNormallyOpen)
        .ReverseLimitSwitchEnabled(true);

    elevatorMotorLeftConfig.softLimit
      .ForwardSoftLimit(ElevatorConstants::extendSoftLimit.value())
      .ForwardSoftLimitEnabled(true);

  //PID Controller 
  /*
  * Configure the closed loop controller. We want to make sure we set the
  * feedback sensor as the primary encoder.
  */
  elevatorMotorLeftConfig.closedLoop
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
  elevatorMotorLeftConfig.encoder.PositionConversionFactor(ElevatorConstants::gearRatio).VelocityConversionFactor(ElevatorConstants::gearRatio);
  elevatorMotorRightConfig.encoder.PositionConversionFactor(ElevatorConstants::gearRatio).VelocityConversionFactor(ElevatorConstants::gearRatio);

// right motor will follow the inverted output of left motor to drive shaft
  bool invertOutput = true;
  elevatorMotorRightConfig.Follow(m_motorLeft, invertOutput);

  /*
   * Apply the configuration to the SPARK MAX.
   *
   * kResetSafeParameters is used to get the SPARK MAX to a known state. This
   * is useful in case the SPARK MAX is replaced.
   *
   * kPersistParameters is used to ensure the configuration is not lost when
   * the SPARK MAX loses power. This is useful for power cycles that may occur mid-operation.*/
   
  m_motorLeft.Configure(elevatorMotorLeftConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                    rev::spark::SparkMax::PersistMode::kPersistParameters);
  m_motorRight.Configure(elevatorMotorRightConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                    rev::spark::SparkMax::PersistMode::kPersistParameters);

  // Reset the position to 0 to start within the range of the soft limits
  m_encoderLeft.SetPosition(ElevatorConstants::resetEncoder.value());
  m_encoderRight.SetPosition(ElevatorConstants::resetEncoder.value());
}

units::meter_t ElevatorSubsystem::GetElevatorHeight() {
  //using left encoder as position reference
  return m_encoderLeft.GetPosition() * ElevatorConstants::distancePerTurn;
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
  m_motorLeft.Set(0);
}

frc2::CommandPtr ElevatorSubsystem::MoveUpCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_motorLeft.Set(0.1);})
         .FinallyDo([this]{
          ResetMotor();
          UpdateSetpoint();
         });
}

frc2::CommandPtr ElevatorSubsystem::MoveDownCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_motorLeft.Set(-0.1);})
         .FinallyDo([this]{
          ResetMotor();
          UpdateSetpoint();
         });
}

frc2::CommandPtr ElevatorSubsystem::MoveToHeightCommand(units::meter_t goal) {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem. */
  return Run([this, goal] {
            m_elevatorGoal = {goal, 0.0_mps }; //stop at goal
            m_elevatorSetpoint = m_trapezoidalProfile.Calculate(ElevatorConstants::kDt, m_elevatorSetpoint, m_elevatorGoal);
            frc::SmartDashboard::PutNumber("trapazoidalSetpoint", m_elevatorSetpoint.position.value());
            m_closedLoopControllerLeft.SetReference(m_elevatorGoal.position.value(), 
                                                rev::spark::SparkLowLevel::ControlType::kPosition,
                                                rev::spark::kSlot0,
                                                m_elevatorFeedforward.Calculate(m_elevatorSetpoint.velocity).value());
            });
}

//To move down supply a negative
frc2::CommandPtr ElevatorSubsystem::MoveUpBy(units::meter_t height) {
      units::meter_t moveGoal = (GetElevatorHeight() + height);
      return MoveToHeightCommand(moveGoal);
}


void ElevatorSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  frc::SmartDashboard::PutNumber("encoderLeftValue", m_encoderLeft.GetPosition());
  frc::SmartDashboard::PutNumber("encoderRightValue", m_encoderRight.GetPosition());
}

void ElevatorSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
