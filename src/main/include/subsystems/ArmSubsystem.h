#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <Constants.h>

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
  frc2::CommandPtr MoveArmToLevelCommand();

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

};




