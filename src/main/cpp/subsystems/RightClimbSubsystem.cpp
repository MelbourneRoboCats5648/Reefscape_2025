#include <numbers>

#include "subsystems/RightClimbSubsystem.h"
// possibly add smart dashboard from example for hard switches
#include <rev/config/SparkMaxConfig.h>

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
  static constexpr units::second_t kDt = 20_ms;

    // Note: These gains are fake, and will have to be tuned for your robot.
    m_motorController.SetPID(1.3, 0.0, 0.7);
  
  rev::spark::SparkMaxConfig motorConfig;

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



void RightClimbSubsystem::Periodic() {
//override - part of example code 
  // Implementation of subsystem periodic method goes here.
  if (m_joystick.GetRawButtonPressed(2)) {
      m_goal = {5_m, 0_mps};
    } else if (m_joystick.GetRawButtonPressed(3)) {
      m_goal = {0_m, 0_mps}


    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    m_setpoint = m_profile.Calculate(kDt, m_setpoint, m_goal);

    // Send setpoint to offboard controller PID
    m_motorController.SetSetpoint(SparkMax::PIDMode::kPosition,
                        m_setpoint.position.value(),
                        m_feedforward.Calculate(m_setpoint.velocity) / 12_V);
  
 }
}
 
void RightClimbSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.

// timed robot code-fixme
  private:
  frc::Joystick m_joystick{1};
  SparkMax m_motorController{1};
  frc::SimpleMotorFeedforward<units::meters> m_feedforward{
      // Note: These gains are fake, and will have to be tuned for your robot.
      1_V, 1.5_V * 1_s / 1_m};

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  frc::TrapezoidProfile<units::meters> m_profile{{1.75_mps, 0.75_mps_sq}};
  frc::TrapezoidProfile<units::meters>::State m_goal;
  frc::TrapezoidProfile<units::meters>::State m_setpoint;
  
}

