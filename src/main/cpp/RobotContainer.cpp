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

using namespace DriveConstants;


RobotContainer::RobotContainer() 
  : m_intakeSubsystem(),
    m_shooterSubsystem(),
    m_intakeAndShootSubsystem(m_intakeSubsystem, m_shooterSubsystem)
{
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();

    m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        units::velocity::meters_per_second_t yspeed = -m_drive.m_yLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetLeftY(), kDeadband) * kMaxSpeed);
        units::velocity::meters_per_second_t xspeed = -m_drive.m_xLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetLeftX(), kDeadband) * kMaxSpeed);
        units::angular_velocity::radians_per_second_t rotspeed = -m_drive.m_rotLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetRightX(), kDeadband) * kMaxAngularSpeed);
        m_drive.Drive(
            // Multiply by max speed to map the joystick unitless inputs to
            // actual units. This will map the [-1, 1] to [max speed backwards,
            // max speed forwards], converting them to actual units.
            yspeed, xspeed, rotspeed, false
            
            //xLimiter.Calculate(m_driverJoystick.GetMagnitude() * DriveConstants::kMaxSpeed),
            //yLimiter.Calculate(m_driverJoystick.GetMagnitude() * DriveConstants::kMaxSpeed),
            //rotLimiter.Calculate(m_driverJoystick.GetDirection() * DriveConstants::kMaxAngularSpeed),
            //m_driverJoystick.(whateverbutton()).WhileTrue(false);
            );
      },
      {&m_drive}));
      
}


void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController.B().WhileTrue(m_drive.StopCommand()); //need to look over controller buttons
  
//intake commands
  m_driverController.LeftBumper().WhileTrue(m_intakeSubsystem.CollectCommand());
  m_driverController.RightBumper().WhileTrue(m_intakeSubsystem.EjectCommand());

  //m_driverController.B().WhileTrue(m_intakeSubsystem.RetractCommand());
  m_driverController.X().WhileTrue(m_intakeSubsystem.ExtendCommand());
  m_driverController.B().WhileTrue(m_shooterSubsystem.ShooterSpeakerAmpCommand());

  //shootercommands
  m_driverController.Y().WhileTrue(m_shooterSubsystem.ShooterSpeakerCommand());
  m_driverController.A().WhileTrue(m_shooterSubsystem.ShooterAmpCommand());
  //onTrue for WithTimeout sequence
  m_driverController.LeftTrigger().OnTrue(m_intakeAndShootSubsystem.PerformIntakeAndShootCommand()); 

  //elevator commands
  m_driverController.LeftStick().WhileTrue(m_elevatorSubsystem.MoveUpToL1Command());
  m_driverController.RightStick().WhileTrue(m_elevatorSubsystem.MoveUpToL2Command());
  //repeated button 
  //m_driverController.LeftTrigger().WhileTrue(m_elevatorSubsystem.MoveUpToL3Command());
  m_driverController.RightTrigger().WhileTrue(m_elevatorSubsystem.MoveDownCommand());

//climb comands
  m_joystick.Button(leftUpButton).OnTrue(std::move(m_leftClimbSubsystem.LeftClimbUpCommand()).Repeatedly().WithTimeout(1.5_s));
  m_joystick.Button(leftDownButton).OnTrue(std::move(m_leftClimbSubsystem.LeftClimbDownCommand()).Repeatedly().WithTimeout(1.5_s));
  m_joystick.Button(rightUpButton).OnTrue(std::move(m_rightClimbSubsystem.RightClimbUpCommand()).Repeatedly().WithTimeout(1.5_s));
  m_joystick.Button(rightDownButton).OnTrue(std::move(m_rightClimbSubsystem.RightClimbDownCommand()).Repeatedly().WithTimeout(1.5_s));

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  //m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
frc2::CommandPtr RobotContainer::GetTestCommand() {
  // An example command will be run in autonomous
  return m_drive.SmartDashboardOutputCommand();
}

