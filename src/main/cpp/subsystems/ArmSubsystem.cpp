#include "subsystems/ArmSubsystem.h"

#include <rev/SparkMax.h>

#include <rev/config/SparkMaxConfig.h>

using namespace ArmConstants;

ArmSubsystem::ArmSubsystem() {
  // Implementation of subsystem constructor goes here.
  rev::spark::SparkMaxConfig armMotorConfig;

  /*
   * Set parameters that will apply to elevator motor.
   */
   armMotorConfig.SmartCurrentLimit(50).SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake); 
   //fixme - test to find out the current limit

      armMotorConfig.limitSwitch
        .ReverseLimitSwitchType(rev::spark::LimitSwitchConfig::Type::kNormallyOpen)
        .ReverseLimitSwitchEnabled(true);

      armMotorConfig.softLimit
        .ForwardSoftLimit(ArmConstants::extendSoftLimit.value())
        .ForwardSoftLimitEnabled(true);

   armMotorConfig.encoder.PositionConversionFactor(ArmConstants::gearRatio).VelocityConversionFactor(ArmConstants::gearRatio);

  /*
   * Apply the configuration to the SPARKs.
   *
   * kResetSafeParameters is used to get the SPARK MAX to a known state. This
   * is useful in case the SPARK MAX is replaced.
   *
   * kPersistParameters is used to ensure the configuration is not lost when
   * the SPARK MAX loses power. This is useful for power cycles that may occur
   * mid-operation.
   */
   m_armMotor.Configure(armMotorConfig,
                        rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                        rev::spark::SparkMax::PersistMode::kPersistParameters);

  m_armEncoder.SetPosition(ArmConstants::resetEncoder.value());
}

void ArmSubsystem::UpdateSetpoint() {  
  m_armSetpoint.position = units::angle::turn_t(m_armEncoder.GetPosition());
  m_armSetpoint.velocity = 0.0_tps; 
}

units::turn_t ArmSubsystem::GetArmAngle(){
  return units::turn_t(m_armEncoder.GetPosition() * ArmConstants::gearRatio);
}

frc::TrapezoidProfile<units::turn>::State& ArmSubsystem::GetSetpoint() {
  return  m_armSetpoint;
} 

frc::TrapezoidProfile<units::turn>::State& ArmSubsystem::GetGoal() {
  return  m_armGoal;
} 

bool ArmSubsystem::IsGoalReached() {
  double errorPosition = std::abs(GetSetpoint().position.value() - GetGoal().position.value());
  double errorVelocity = std::abs(GetSetpoint().velocity.value() - GetGoal().velocity.value());

  if ((errorPosition <= kArmPositionToleranceTurns) && (errorVelocity <= kArmVelocityTolerancePerSecond)){
    return true;
  }
  else {
    return false;
  }

}

frc2::CommandPtr ArmSubsystem::MoveUpCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_armMotor.Set(-0.1);})
          .FinallyDo([this]{m_armMotor.Set(0);});
}

frc2::CommandPtr ArmSubsystem::MoveDownCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_armMotor.Set(0.1);})
          .FinallyDo([this]{m_armMotor.Set(0);});
}

frc2::CommandPtr ArmSubsystem::MoveToAngleCommand(units::turn_t goal) {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem. */
  return Run([this, goal] {
            m_armGoal = {goal, 0.0_tps }; //stop at goal
            m_armSetpoint = m_trapezoidalProfile.Calculate(ArmConstants::kDt, m_armSetpoint, m_armGoal);

            frc::SmartDashboard::PutNumber("trapazoidalSetpoint", m_armSetpoint.position.value());

            m_closedLoopController.SetReference(m_armGoal.position.value(), rev::spark::SparkLowLevel::ControlType::kPosition);})
            .Until([this] {return IsGoalReached();});
}

//To move down supply a negative
frc2::CommandPtr ArmSubsystem::RotateBy(units::turn_t angle) {
      units::turn_t moveGoal = (GetArmAngle() + angle);
      return MoveToAngleCommand(moveGoal);
}

//stops motor
void ArmSubsystem::StopMotor() {
  m_armMotor.Set(0.0);
}

//reset encoder
void ArmSubsystem::ResetEncoder() {
  m_armEncoder.SetPosition(0.0);
}

void ArmSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  frc::SmartDashboard::PutNumber("armEncoderValue", m_armEncoder.GetPosition());
}

void ArmSubsystem::OnLimitSwitchActivation() {
      if(m_limitSwitchArm.Get()) {
      StopMotor();
      ResetEncoder();
    }
}

void ArmSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
