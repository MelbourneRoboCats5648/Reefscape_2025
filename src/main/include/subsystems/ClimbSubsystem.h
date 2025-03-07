#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <Constants.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/ArmFeedforward.h>

class ClimbSubsystem : public frc2::SubsystemBase {
  private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Initialize the motor
  rev::spark::SparkMax m_climbMotor{CAN_Constants::kClimbCAN_ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkRelativeEncoder m_climbEncoder = m_climbMotor.GetEncoder();

  public:
  ClimbSubsystem();

  //climb command factory method.
  void MoveClimb(double speed);
  frc2::CommandPtr MoveUpCommand();
  frc2::CommandPtr MoveDownCommand();
  frc2::CommandPtr MoveToAngleCommand(units::turn_t goal);
  frc2::CommandPtr RotateBy(units::turn_t angle);

  void StopMotor();
  void ResetEncoder();
  
  void UpdateSetpoint();
  units::turn_t GetClimbAngle();
  frc::TrapezoidProfile<units::turn>::State& GetSetpoint();
  frc::TrapezoidProfile<units::turn>::State& GetGoal();
  bool IsGoalReached();
  
  void SetpointControl(); // control using setpoint
  frc2::CommandPtr SetpointControlCommand();

  //Will be called periodically whenever the CommandScheduler runs.
  void Periodic() override;

  //Will be called periodically whenever the CommandScheduler runs during simulation.
  void SimulationPeriodic() override;

  private:
  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.

  frc::TrapezoidProfile<units::turn> m_trapezoidalProfile{{ClimbConstants::maximumVelocity, ClimbConstants::maximumAcceleration}};
  frc::TrapezoidProfile<units::turn>::State m_climbGoal{ClimbConstants::resetEncoder, 0.0_tps};
  frc::TrapezoidProfile<units::turn>::State m_climbSetpoint{ClimbConstants::resetEncoder, 0.0_tps};
  rev::spark::SparkClosedLoopController m_closedLoopController = m_climbMotor.GetClosedLoopController();

  frc::ArmFeedforward m_climbFeedforward{ClimbConstants::kS, ClimbConstants::kG, ClimbConstants::kV, ClimbConstants::kA};

};




