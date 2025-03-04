#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <Constants.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <frc/controller/ArmFeedforward.h>

class ArmSubsystem : public frc2::SubsystemBase {
  private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Initialize the motor
  rev::spark::SparkMax m_armMotor{CAN_Constants::kElevatorArmMotorCAN_ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkRelativeEncoder m_armEncoder = m_armMotor.GetEncoder();

  //digital input
  frc::DigitalInput m_limitSwitchArm{ArmConstants::k_limitSwitchArmPin};

  public:
  ArmSubsystem();

  //arm command factory method.
  void MoveArm(double speed);
  frc2::CommandPtr MoveUpCommand();
  frc2::CommandPtr MoveDownCommand();
  frc2::CommandPtr MoveToAngleCommand(units::turn_t goal);
  frc2::CommandPtr RotateBy(units::turn_t angle);

  //todo - figure out commands for arm

  void StopMotor();
  void ResetEncoder();
  
  void UpdateSetpoint();
  units::turn_t GetArmAngle();
  frc::TrapezoidProfile<units::turn>::State& GetSetpoint();
  frc::TrapezoidProfile<units::turn>::State& GetGoal();
  bool IsGoalReached();
  
  void SetpointControl(); // control using setpoint
  frc2::CommandPtr SetpointControlCommand();

  void OnLimitSwitchActivation();

  //Will be called periodically whenever the CommandScheduler runs.
  void Periodic() override;

  //Will be called periodically whenever the CommandScheduler runs during simulation.
  void SimulationPeriodic() override;

  private:
  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.

  frc::TrapezoidProfile<units::turn> m_trapezoidalProfile{ArmConstants::trapezoidProfile};
  frc::TrapezoidProfile<units::turn>::State m_armGoal{ArmConstants::resetEncoder, 0.0_tps};
  frc::TrapezoidProfile<units::turn>::State m_armSetpoint{ArmConstants::resetEncoder, 0.0_tps};
  rev::spark::SparkClosedLoopController m_closedLoopController = m_armMotor.GetClosedLoopController();

  frc::ArmFeedforward m_armFeedforward{ArmConstants::kS, ArmConstants::kG, ArmConstants::kV, ArmConstants::kA};

};




