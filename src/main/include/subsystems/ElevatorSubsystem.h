#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc2/command/button/CommandXboxController.h>

// PWM Ports
const int kMotorControllerPort = 2;


class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorSubsystem();

  /**
   * Elevator command factory method.
   */
  frc2::CommandPtr GoUpToL1Command();
  frc2::CommandPtr GoUpToL2Command();
  frc2::CommandPtr GoUpToL3Command();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
    frc::PWMSparkMax m_motorController{kMotorControllerPort};
};




