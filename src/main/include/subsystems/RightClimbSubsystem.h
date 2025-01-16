// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/VictorSP.h>
#include <frc/Joystick.h>
#include <rev/SparkMax.h>

using namespace rev::spark;

//FRC Motor Ports
const int motorClimbRightPort= 1;

//Motor ID
const int motorClimbRightID = 1;

// Speeds
const double rightClimbUpSpeed = 1.0; //was 0.25
const double rightClimbDownSpeed = -1.0;

// Joystick buttons
const int rightUpButton = 6;
const int rightDownButton = 4;

// Soft Limits
const int  extendSoftLimit = 50;
const int  retractSoftLimit= -50;

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
  
//old frc 2024 season motors redundant (need to change to spark)
          frc::VictorSP m_motorClimbRight{motorClimbRightPort};

// Spark Brushless 
          SparkMax m_motor{motorClimbRightID, SparkMax::MotorType::kBrushless};
          SparkRelativeEncoder m_encoder = m_motor.GetEncoder();

};
