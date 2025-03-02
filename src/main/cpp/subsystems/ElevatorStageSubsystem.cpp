#include <subsystems/ElevatorStageSubsystem.h>

ElevatorStageSubsystem::ElevatorStageSubsystem(
  std::string name,
  units::meter_t minLimit, units::meter_t maxLimit,
  units::meter_t initHeight, units::meter_t resetHeight,
  units::meter_t distancePerTurn,
  PIDConstants pidConst, ElevatorFeedforwardConstants ffConst,
  int canID, int limitSwitchPin, int followerID
) : m_name(name),
    m_limitSwitch(limitSwitchPin),
    m_motor(canID, rev::spark::SparkMax::MotorType::kBrushless),
    m_encoder(m_motor.GetEncoder()),
    m_closedLoopController(m_motor.GetClosedLoopController()),
    m_goal({initHeight, 0.0_mps}),
    m_setpoint(m_goal),
    m_feedforward(ffConst.kS, ffConst.kG, ffConst.kV, ffConst.kA),
    m_minLimit(minLimit), m_maxLimit(maxLimit), m_resetHeight(resetHeight),
    m_gearRatio(ElevatorConstants::gearBoxGearRatio * distancePerTurn.value())
 {
    rev::spark::SparkMaxConfig motorConfig;
    motorConfig
      .SmartCurrentLimit(ElevatorConstants::kCurrentLimit)
      .SetIdleMode(rev::spark::SparkMaxConfig::kCoast);
    motorConfig.softLimit
      .ForwardSoftLimit(maxLimit.value()).ForwardSoftLimitEnabled(true)
      .ReverseSoftLimit(minLimit.value()).ReverseSoftLimitEnabled(true);
    motorConfig.closedLoop
      .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      .P(pidConst.kP).I(pidConst.kI).D(pidConst.kD)
      .OutputRange(-ElevatorConstants::maxOutput, ElevatorConstants::maxOutput);
    motorConfig.encoder
      .PositionConversionFactor(m_gearRatio)
      .VelocityConversionFactor(m_gearRatio);
    m_motor.Configure(
      motorConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kPersistParameters
    );
    m_encoder.SetPosition(initHeight.value());

    if (followerID >= 0) {
      /* we have a follower motor controller, let's set it up */
      rev::spark::SparkMax followerMotor(followerID, rev::spark::SparkMax::MotorType::kBrushless);
      rev::spark::SparkMaxConfig followerConfig;
      followerConfig
        .SmartCurrentLimit(ElevatorConstants::kCurrentLimit)
        .SetIdleMode(rev::spark::SparkMaxConfig::kCoast)
        .Follow(m_motor, true); // invert = true
      followerMotor.Configure(
        followerConfig,
        rev::spark::SparkMax::ResetMode::kResetSafeParameters,
        rev::spark::SparkMax::PersistMode::kPersistParameters
      );
    }

    SetDefaultCommand(SetpointControlCommand());
}

units::meter_t ElevatorStageSubsystem::GetHeight() {
  return units::meter_t(m_encoder.GetPosition());
}

units::meters_per_second_t ElevatorStageSubsystem::GetVelocity() {
  return units::meters_per_second_t(m_encoder.GetVelocity() / 60.0); // GetVelocity() returns metres per minute
}

bool ElevatorStageSubsystem::IsGoalReached() {
  auto posErr = units::math::abs(GetHeight() - m_goal.position);
  auto velErr = units::math::abs(GetVelocity() - m_goal.velocity);
  
  if ((posErr <= ElevatorConstants::kElevatorPositionTolerance) &&
      (velErr <= ElevatorConstants::kElevatorVelocityTolerance))
  {
    return true;
  }
  else {
    return false;
  }
}

void ElevatorStageSubsystem::SetpointControl() {
  m_closedLoopController.SetReference(m_setpoint.position.value(), 
                                      rev::spark::SparkLowLevel::ControlType::kPosition,
                                      rev::spark::kSlot0,
                                      m_feedforward.Calculate(m_setpoint.velocity).value());
}

frc2::CommandPtr ElevatorStageSubsystem::SetpointControlCommand() {
  return Run([this] { SetpointControl(); });
}

void ElevatorStageSubsystem::UpdateSetpoint() {
  m_setpoint = { GetHeight(), 0.0_mps };
}

void ElevatorStageSubsystem::ResetMotor() {
  m_motor.Set(0);
}

void ElevatorStageSubsystem::ResetEncoder() {
  m_encoder.SetPosition(m_resetHeight.value());
}

void ElevatorStageSubsystem::OnLimitSwitchActivation() {
  ResetMotor();
  ResetEncoder();
}

frc2::CommandPtr ElevatorStageSubsystem::MoveUpCommand() {
  return Run([this] { m_motor.Set(0.1); })
         .FinallyDo([this] {
            ResetMotor();
            UpdateSetpoint();
         });
}

frc2::CommandPtr ElevatorStageSubsystem::MoveDownCommand() {
  return
    Run([this] { m_motor.Set(-0.1); })
    .FinallyDo([this] {
      ResetMotor();
      UpdateSetpoint();
    });
}

frc2::CommandPtr ElevatorStageSubsystem::MoveToHeightCommand(units::meter_t heightGoal) {
  return
    Run([this, heightGoal] {
      m_goal = { heightGoal, 0.0_mps };
      m_setpoint = m_trapezoidalProfile.Calculate(ElevatorConstants::kDt, m_setpoint, m_goal);
      SetpointControl();
    })
    .Until([this] { return IsGoalReached(); })
    .FinallyDo([this] { UpdateSetpoint(); });
}

frc2::CommandPtr ElevatorStageSubsystem::MoveUpBy(units::meter_t height) {
  units::meter_t moveGoal = (GetHeight() + height);
  return MoveToHeightCommand(moveGoal);
}

void ElevatorStageSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber(m_name + "/setpoint/position", m_setpoint.position.value());
  frc::SmartDashboard::PutNumber(m_name + "/setpoint/velocity", m_setpoint.velocity.value());
  frc::SmartDashboard::PutNumber(m_name + "/goal/position", m_goal.position.value());
  frc::SmartDashboard::PutNumber(m_name + "/goal/velocity", m_goal.velocity.value());
  frc::SmartDashboard::PutNumber(m_name + "/encoder/position", GetHeight().value());
  frc::SmartDashboard::PutNumber(m_name + "/encoder/velocity", GetVelocity().value());
}

void ElevatorStageSubsystem::SimulationPeriodic() {
  
}
