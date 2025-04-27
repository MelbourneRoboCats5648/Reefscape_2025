#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <Constants.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/Servo.h>


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
  frc2::CommandPtr LockClimbCommand();
  frc2::CommandPtr ReleaseClimbCommand();
  frc2::CommandPtr MoveClimbCommand(double speed);

  void SetpointControl();

  void StopMotor();
  void LockRatchet();
  void ReleaseRatchet();

  void ResetEncoder();

  units::turn_t GetClimbAngle();
  units::turns_per_second_t GetClimbVelocity();
  bool IsGoalReached();

  //Will be called periodically whenever the CommandScheduler runs.
  void Periodic() override;

  //Will be called periodically whenever the CommandScheduler runs during simulation.
  void SimulationPeriodic() override;

  private:
  frc::Servo ratchetServo {ClimbConstants::servoPWM_Pin};

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.

  frc::ProfiledPIDController<units::turn> m_controller{
    ClimbConstants::kP, ClimbConstants::kI, ClimbConstants::kD,
    {ClimbConstants::maximumVelocity, ClimbConstants::maximumAcceleration},
    ClimbConstants::kDt
  };
  frc::ArmFeedforward m_climbFeedforward{ClimbConstants::kS, ClimbConstants::kG, ClimbConstants::kV, ClimbConstants::kA};

};




