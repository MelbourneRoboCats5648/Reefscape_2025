// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandJoystick.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ClimbSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/ElevatorAndArmSubsystem.h"
#include "subsystems/VisionSubsystem.h"

/* This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here. */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetInitCommand();
  frc2::CommandPtr GetAutonomousCommand();
  frc2::CommandPtr GetTestCommand();

 private:
  double GetMechLeftY();
  double GetMechRightY();
  
  void ConfigureBindings();
  

  double ScaleJoystickInput(double input);
 
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_driverController{OperatorConstants::kDriverControllerPort};
  frc2::CommandXboxController m_mechController{OperatorConstants::kMechControllerPort};  

  // The robot's subsystems are defined here...
  DriveSubsystem m_drive;
  ClimbSubsystem m_climbSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  ArmSubsystem m_armSubsystem;
  //this subsystem relies on the two subsystems above
  ElevatorAndArmSubsystem m_elevatorAndArmSubsystem;
  VisionSubsystem m_visionSubsystem;
};
