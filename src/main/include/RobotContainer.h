// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandJoystick.h>

#include "Constants.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LeftClimbSubsystem.h"
#include "subsystems/RightClimbSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"


/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_driverController{OperatorConstants::kDriverControllerPort};
  frc2::CommandJoystick m_joystick{OperatorConstants::kDriveJoystickPort};

  // The robot's subsystems are defined here...
  ExampleSubsystem m_subsystem;
  IntakeSubsystem m_intakeSubsystem;
  LeftClimbSubsystem m_leftClimbSubsystem;
  RightClimbSubsystem m_rightClimbSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  //ElevatorSubsystem m_elevatorSubsystem;

  units::turn_t m_climbGoalL1 = 1.0_tr; // TODO: we may want to move this to Constants.h or somethingm_
  units::turn_t m_climbGoalRetract = 0.0_tr; 

  void ConfigureBindings();
};
