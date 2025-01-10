// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/VictorSP.h>

//Port Numbers
const int k_motorShooterLeftPort = 4;
const int k_motorShooterRightPort = 5;

//Speed Constants
const double speakerShooterSpeed = 1.0;
const double ampShooterSpeed = 0.2;

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  /** 
   * Example command factory method.
   */
  frc2::CommandPtr ShooterAmpCommand();
  frc2::CommandPtr ShooterSpeakerCommand();
  
  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  bool ExampleCondition();

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
  frc::VictorSP m_motorShooterLeft{k_motorShooterLeftPort};
  frc::VictorSP m_motorShooterRight{k_motorShooterRightPort};
};
