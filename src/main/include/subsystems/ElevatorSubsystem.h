#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <Constants.h>

class ElevatorSubsystem : public frc2::SubsystemBase {
  private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Initialize the motor
  rev::spark::SparkMax m_elevatorLeftLiftMotor{CAN_Constants::kElevatorLeftMotorCAN_ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_elevatorRightLiftMotor{CAN_Constants::kElevatorRightMotorCAN_ID, rev::spark::SparkMax::MotorType::kBrushless};
  public:
  ElevatorSubsystem();

  /**
   * Elevator command factory method.
   */
  frc2::CommandPtr MoveUpToL1Command();
  frc2::CommandPtr MoveUpToL2Command();
  frc2::CommandPtr MoveUpToL3Command();
  frc2::CommandPtr MoveDownCommand();


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




