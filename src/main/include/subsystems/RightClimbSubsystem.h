// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/VictorSP.h>
#include <frc/Joystick.h>

// PWM ports
const int motorClimbRightPort = 0;

// Speeds
const double climbUpSpeed = 1.0; //was 0.25
const double climbDownSpeed = -1.0;

// Joystick buttons
const int rightUpButton = 6;
const int rightDownButton = 4;

class RightClimbSubsystem : public frc2::SubsystemBase {
 public:
  RightClimbSubsystem();

  /**
   * Command factory method.
   */
  frc2::CommandPtr RightClimbUpCommand();
  frc2::CommandPtr RightClimbDownCommand();


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
          frc::VictorSP m_motorClimbRight{motorClimbRightPort};

};
