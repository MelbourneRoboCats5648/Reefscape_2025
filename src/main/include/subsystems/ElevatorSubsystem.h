#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <rev/SparkMax.h>

#include <frc/Joystick.h>


// PWM Ports
const int kMotorControllerPort = 6;

using namespace rev::spark;

class ElevatorSubsystem : public frc2::SubsystemBase {
  private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
    frc::PWMSparkMax m_motorController{kMotorControllerPort};

    rev::spark::SparkMaxConfig globalConfig;
    rev::spark::SparkMaxConfig rightLeaderConfig;
    rev::spark::SparkMaxConfig leftFollowerConfig;
    rev::spark::SparkMaxConfig rightFollowerConfig;


  // Initialize the SPARKs
  SparkMax m_leftLeader{1, SparkMax::MotorType::kBrushless};
  SparkMax m_leftFollower{2, SparkMax::MotorType::kBrushless};
  SparkMax m_rightLeader{3, SparkMax::MotorType::kBrushless};
  SparkMax m_rightFollower{4, SparkMax::MotorType::kBrushless};

  // Initialize joystick
  frc::Joystick joystick{0};

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

};



