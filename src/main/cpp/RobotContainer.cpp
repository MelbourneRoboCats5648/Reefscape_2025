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

  m_driverController.LeftBumper().WhileTrue(m_intakeSubsystem.CollectCommand());
  m_driverController.RightBumper().WhileTrue(m_intakeSubsystem.EjectCommand());
  m_driverController.B().WhileTrue(m_intakeSubsystem.RetractCommand());
  m_driverController.X().WhileTrue(m_intakeSubsystem.ExtendCommand());
  m_driverController.Y().WhileTrue(m_shooterSubsystem.ShooterSpeakerCommand());
  m_driverController.A().WhileTrue(m_shooterSubsystem.ShooterAmpCommand());

  m_joystick.Button(leftUpButton).WhileTrue(m_leftClimbSubsystem.LeftClimbUpCommand());
  m_joystick.Button(leftDownButton).WhileTrue(m_leftClimbSubsystem.LeftClimbDownCommand());
  m_joystick.Button(rightUpButton).WhileTrue(m_rightClimbSubsystem.RightClimbUpCommand());
  m_joystick.Button(rightDownButton).WhileTrue(m_rightClimbSubsystem.RightClimbDownCommand());

  m_driverController.B().WhileTrue(m_elevatorSubsystem.MoveUpToL1Command());
  m_driverController.X().WhileTrue(m_elevatorSubsystem.MoveUpToL2Command());
  m_driverController.Y().WhileTrue(m_elevatorSubsystem.MoveUpToL3Command());
  m_driverController.A().WhileTrue(m_elevatorSubsystem.MoveDownCommand());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
