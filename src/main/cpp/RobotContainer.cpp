// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  m_driverController.LeftBumper().OnTrue(m_intakeSubsystem.CollectCommand());
  m_driverController.RightBumper().OnTrue(m_intakeSubsystem.EjectCommand());
  m_driverController.B().OnTrue(m_intakeSubsystem.RetractCommand());
  m_driverController.X().OnTrue(m_intakeSubsystem.ExtendCommand());
  m_joystick.GetRawButton(leftUpButton).Ontrue(m_leftClimbSubsystem.LeftClimbUpCommand());
  m_joystick.GetRawButton(leftDownButton).OnTrue(m_leftClimbSubsystem.LeftClimbDownCommand());
  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
