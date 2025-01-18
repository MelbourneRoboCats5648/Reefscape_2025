#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <Constants.h>

// PWM Ports
using namespace rev::spark;

class ElevatorSubsystem : public frc2::SubsystemBase {
  private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Initialize the motor
  SparkMax m_elevatorLiftMotor{CAN_Constants::kElevatorMotorCAN_ID, SparkMax::MotorType::kBrushless};

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




