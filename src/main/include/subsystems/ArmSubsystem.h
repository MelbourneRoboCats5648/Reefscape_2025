#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <Constants.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

class ArmSubsystem : public frc2::SubsystemBase {
  private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Initialize the motor
  rev::spark::SparkMax m_elevatorArmMotor{CAN_Constants::kElevatorArmMotorCAN_ID, rev::spark::SparkMax::MotorType::kBrushless};

  public:
  ArmSubsystem();

  /**
   * arm command factory method.
   */
  frc2::CommandPtr MoveArmUpCommand();
  frc2::CommandPtr MoveArmDownCommand();
  frc2::CommandPtr MoveArmToLevelCommand(units::turn_t goal);


//todo - figure out commands for arm

  void StopMotor();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
frc::TrapezoidProfile<units::turn> m_trapezoidalProfile{{ArmConstants::maximumVelocity, ArmConstants::maximumAcceleration}};
  frc::TrapezoidProfile<units::turn>::State m_ArmGoal;
  frc::TrapezoidProfile<units::turn>::State m_ArmSetpoint;
  rev::spark::SparkClosedLoopController m_closedLoopController = m_elevatorArmMotor.GetClosedLoopController();

};




