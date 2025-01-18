#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>

#include <frc2/command/button/CommandXboxController.h>


// PWM Ports
const int kMotorControllerPort = 6;

using namespace rev::spark;

class ElevatorSubsystem : public frc2::SubsystemBase {
  private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Initialize the SPARKs
  SparkMax m_elevatorLiftMotor{1, SparkMax::MotorType::kBrushless};

  // Initialize the controller
  frc2::CommandXboxController xboxController {0}; // need to be fixed - only have one physical xbox controller

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




