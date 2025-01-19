// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/VictorSP.h>
#include <frc/Joystick.h>
#include <rev/SparkMax.h>
#include <frc/controller/PIDController.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

using namespace rev::spark;

//FRC Motor Ports

const int motorClimbRightPort= 0;

//Motor ID
//plan to change with more Spark motors added
const int motorClimbRightID = 1;

// Speeds
const double rightClimbUpSpeed = 1.0; //was 0.25
const double rightClimbDownSpeed = -1.0;

// Joystick buttons
const int rightUpButton = 6;
const int rightDownButton = 4;

// Soft Limits
//plant to change from example base limits when limits are changed
const int  extendSoftLimit = 50;
const int  retractSoftLimit= -50;

 
//PID Controller

//PID Profile] 


class RightClimbSubsystem : public frc2::SubsystemBase {
 public:
  RightClimbSubsystem();

  /**
   * Command factory method.
   */
  frc2::CommandPtr RightClimbUpCommand();
  frc2::CommandPtr RightClimbDownCommand();
  
    
  [[nodiscard]]
  frc2::CommandPtr RightClimbCommand(units::turns_per_second_t setpoint);



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

// Spark components
//plan to add motors to hard switches
          SparkMax m_motor{motorClimbRightID, SparkMax::MotorType::kBrushless};
          SparkRelativeEncoder m_encoder = m_motor.GetEncoder();
          

};
