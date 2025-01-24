// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"

#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() 
{
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
//intake subsystem commands
  m_driverController.LeftBumper().WhileTrue(m_intakeSubsystem.CollectCommand());
  m_driverController.RightBumper().WhileTrue(m_intakeSubsystem.EjectCommand());
  m_driverController.B().WhileTrue(m_intakeSubsystem.RetractCommand());
  m_driverController.X().WhileTrue(m_intakeSubsystem.ExtendCommand());

// shooter susbsystem commands
  m_driverController.Y().WhileTrue(m_shooterSubsystem.ShooterSpeakerCommand());
  m_driverController.A().WhileTrue(m_shooterSubsystem.ShooterAmpCommand());

// elevator subsystem commands
  //m_driverController.LeftStick().WhileTrue(m_elevatorSubsystem.MoveUpToL1Command(ElevatorConstants::level1Goal)); //set three for now will change
  //m_driverController.RightStick().WhileTrue(m_elevatorSubsystem.MoveUpToL2Command(ElevatorConstants::level2Goal)); - deleting soon
  //m_driverController.LeftTrigger().WhileTrue(m_elevatorSubsystem.MoveUpToL3Command(ElevatorConstants::level3Goal));
  m_driverController.RightTrigger().WhileTrue(m_elevatorSubsystem.MoveDownCommand());

//climb susbsystem commands
  m_joystick.Button(leftUpButton).OnTrue(std::move(m_leftClimbSubsystem.LeftClimbUpCommand()).Repeatedly().WithTimeout(1.5_s));
  m_joystick.Button(leftDownButton).OnTrue(std::move(m_leftClimbSubsystem.LeftClimbDownCommand()).Repeatedly().WithTimeout(1.5_s));
  //m_joystick.Button(rightUpButton).OnTrue(std::move(m_rightClimbSubsystem.RightClimbUpCommand()).Repeatedly().WithTimeout(1.5_s));
  //m_joystick.Button(rightDownButton).OnTrue(std::move(m_rightClimbSubsystem.RightClimbDownCommand()).Repeatedly().WithTimeout(1.5_s));

//PID right climb subsystem command
m_driverController.Button(leftDownButton).OnTrue(m_elevatorSubsystem.MoveToLevelCommand(ElevatorConstants::climbGoalRetract));
m_driverController.Button(leftUpButton).OnTrue(m_elevatorSubsystem.MoveToLevelCommand(ElevatorConstants::level1Goal));
m_driverController.Button(leftUpButton).OnTrue(m_elevatorSubsystem.MoveToLevelCommand(ElevatorConstants::level2Goal));
m_driverController.Button(leftUpButton).OnTrue(m_elevatorSubsystem.MoveToLevelCommand(ElevatorConstants::level3Goal));

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  //m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() 
{
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}