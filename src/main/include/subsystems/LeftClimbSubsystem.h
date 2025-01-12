// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/Joystick.h>
#include <frc/motorcontrol/VictorSP.h>

// PWM Ports
const int motorClimbLeftPort = 1;

// Speeds
const double leftClimbUpSpeed = 1.0; //was 0.25
const double leftClimbDownSpeed = -1.0;

// Joystick buttons
const int leftUpButton = 5;
const int leftDownButton = 3;

class LeftClimbSubsystem : public frc2::SubsystemBase {
 public:
  LeftClimbSubsystem();

  /**
   * LeftClimb command factory method.
   */
  frc2::CommandPtr LeftClimbUpCommand();
  frc2::CommandPtr LeftClimbDownCommand();

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
      frc::VictorSP m_motorClimbLeft{motorClimbLeftPort};

};

