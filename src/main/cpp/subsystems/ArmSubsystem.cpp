#include "subsystems/ArmSubsystem.h"

#include <rev/SparkMax.h>

#include <rev/config/SparkMaxConfig.h>

ArmSubsystem::ArmSubsystem() {
  // Implementation of subsystem constructor goes here.
  rev::spark::SparkMaxConfig armMotorConfig;

  /*
   * Set parameters that will apply to elevator motor.
   */
   armMotorConfig.SmartCurrentLimit(50).SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake); 
   //fixme - test to find out the current limit

      armMotorConfig.limitSwitch
        //ForwardLimitSwitchType(rev::spark::LimitSwitchConfig::Type::kNormallyOpen)
        //ForwardLimitSwitchEnabled(true);
        .ReverseLimitSwitchType(rev::spark::LimitSwitchConfig::Type::kNormallyOpen)
        .ReverseLimitSwitchEnabled(true);

    //  // Set the soft limits to stop the motor at -50 and 50 rotations
    //  //will alter constants 
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


frc2::CommandPtr ArmSubsystem::MoveUpCommand() {
  // Inline construction of command goes here.
  return Run([this] {m_armMotor.Set(-0.1);})
          .FinallyDo([this]{m_armMotor.Set(0);})
;
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
            frc::TrapezoidProfile<units::turn>::State goalState = {goal, 0.0_tps }; //stop at goal
            m_armSetpoint = m_trapezoidalProfile.Calculate(ArmConstants::kDt, m_armSetpoint, goalState);

            frc::SmartDashboard::PutNumber("trapazoidalSetpoint", m_armSetpoint.position.value());

            m_closedLoopController.SetReference(goalState.position.value(), rev::spark::SparkLowLevel::ControlType::kPosition);
            });
}

//stops motor
void ArmSubsystem::StopMotor() {
  m_armMotor.Set(0.0);
}

void ArmSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  frc::SmartDashboard::PutNumber("encoderValue", m_armEncoder.GetPosition());
}

void ArmSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
