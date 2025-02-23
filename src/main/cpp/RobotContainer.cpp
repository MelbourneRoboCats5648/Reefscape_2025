// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"


#include <frc2/command/button/Trigger.h>

//copied from frc example code
#include <utility>
#include <frc/controller/PIDController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>

#include "commands/Autos.h"

#include "commands/ExampleCommand.h"

#include <frc/filter/SlewRateLimiter.h>
#include <frc/Joystick.h>

using namespace DriveConstants;

RobotContainer::RobotContainer()
   :m_shooterSubsystem(),
    m_elevatorSubsystem(),
    m_armSubsystem(),
    m_elevatorAndArmSubsystem(m_elevatorSubsystem, m_armSubsystem)
{

  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  switch(General::KBuildSeason)
  {
    case (BuildSeason::Reefscape):
    {
      ConfigureBindings();
      break;
    }
    case (BuildSeason::Crescendo):
    {
      Configure2024Bindings();
      break;
    }
  }

  m_drive.SetDefaultCommand(frc2::RunCommand(
    [this] {
      units::velocity::meters_per_second_t yspeed = -m_drive.m_yLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetLeftY(), kDeadband) * kMaxSpeed);
      units::velocity::meters_per_second_t xspeed = -m_drive.m_xLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetLeftX(), kDeadband) * kMaxSpeed);
      units::angular_velocity::radians_per_second_t rotspeed = -m_drive.m_rotLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetRightX(), kDeadband) * kMaxAngularSpeed);
      m_drive.Drive(
          // Multiply by max speed to map the joystick unitless inputs to
          // actual units. This will map the [-1, 1] to [max speed backwards,
          // max speed forwards], converting them to actual units.
          yspeed, xspeed, rotspeed, false );
    },
    {&m_drive}));

}


void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  //PID elevator subsystem command
  m_driverController.A().OnTrue(m_elevatorAndArmSubsystem.MoveToLevel(Level::L1));
  m_driverController.X().OnTrue(m_elevatorAndArmSubsystem.MoveToLevel(Level::L2));
  m_driverController.Y().OnTrue(m_elevatorAndArmSubsystem.MoveToLevel(Level::L3));
  m_driverController.B().OnTrue(m_elevatorAndArmSubsystem.MoveToLevel(Level::L4));

  //Collect Command
  m_driverController.LeftBumper().OnTrue(m_elevatorAndArmSubsystem.CollectCoral());

  //Place on Reef Command
  m_driverController.RightBumper().OnTrue(m_elevatorAndArmSubsystem.PlaceCoral());

// PID climb subsystem command (temporarily disabling LeftClimbUpCommand and LeftClimbDownCommand to test it)
  m_joystick.Button(LeftClimbConstants::leftUpButton).OnTrue(m_leftClimbSubsystem.LeftClimbCommand(GoalConstants::m_climbGoalL1));
  m_joystick.Button(LeftClimbConstants::leftDownButton).OnTrue(m_leftClimbSubsystem.LeftClimbCommand(GoalConstants::m_climbGoalRetract));

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  //m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
  m_driverController.LeftTrigger().WhileTrue(m_elevatorSubsystem.MoveUpCommand());
  m_driverController.RightTrigger().WhileTrue(m_elevatorSubsystem.MoveDownCommand());

}

void RobotContainer::Configure2024Bindings() {
  // Configure your trigger bindings here
  
  //drivetrain commands
  m_driverController.B().WhileTrue(m_drive.StopCommand()); 

  //shootercommands
  m_driverController.Y().WhileTrue(m_shooterSubsystem.ShooterSpeakerCommand());
  m_driverController.A().WhileTrue(m_shooterSubsystem.ShooterAmpCommand());
  //m_driverController.B().WhileTrue(m_shooterSubsystem.ShooterSpeakerAmpCommand());
 

  //left climb commands
  m_joystick.Button(LeftClimbConstants::leftUpButton).OnTrue(std::move(m_leftClimbSubsystem.LeftClimbUpCommand()).Repeatedly().WithTimeout(1.5_s));
  m_joystick.Button(LeftClimbConstants::leftDownButton).OnTrue(std::move(m_leftClimbSubsystem.LeftClimbDownCommand()).Repeatedly().WithTimeout(1.5_s));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() 
{
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
frc2::CommandPtr RobotContainer::GetTestCommand() {
  // An example command will be run in autonomous
  return m_drive.SmartDashboardOutputCommand();
}

