// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"


#include <frc2/command/button/Trigger.h>

//copied from frc example code
#include <utility>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

#include <frc/filter/SlewRateLimiter.h>
#include <frc/Joystick.h>

#include <frc2/command/button/Trigger.h>

using namespace DriveConstants;


RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();

    m_driveSubsystem.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_driveSubsystem.Drive(
            // Multiply by max speed to map the joystick unitless inputs to
            // actual units. This will map the [-1, 1] to [max speed backwards,
            // max speed forwards], converting them to actual units.
            m_driverController.GetLeftY() * DriveConstants::kMaxSpeed, //xspeed
            m_driverController.GetLeftX() * DriveConstants::kMaxSpeed, //yspeed
            m_driverController.GetRightX() * DriveConstants::kMaxAngularSpeed, //rotation
            false //fieldrelative
            
            //xLimiter.Calculate(m_driverJoystick.GetMagnitude() * DriveConstants::kMaxSpeed),
            //yLimiter.Calculate(m_driverJoystick.GetMagnitude() * DriveConstants::kMaxSpeed),
            //rotLimiter.Calculate(m_driverJoystick.GetDirection() * DriveConstants::kMaxAngularSpeed),
            //m_driverJoystick.(whateverbutton()).WhileTrue(false);
            );
      },
      {&m_driveSubsystem}));
      
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
  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());

  m_visionSubsystem.m_aprilTagIDTrigger.WhileTrue(autos::VisionDrive(&m_visionSubsystem, &m_driveSubsystem));
  
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
