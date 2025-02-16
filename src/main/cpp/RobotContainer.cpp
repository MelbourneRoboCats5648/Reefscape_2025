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

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

#include <frc/filter/SlewRateLimiter.h>
#include <frc/Joystick.h>

#include <pathplanner/lib/commands/PathPlannerAuto.h>

using namespace DriveConstants;
using namespace pathplanner;


RobotContainer::RobotContainer() 
  : m_intakeSubsystem(),
    m_shooterSubsystem(),
    m_intakeAndShootSubsystem(m_intakeSubsystem, m_shooterSubsystem)
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
  m_visionSubsystem.m_aprilTagIDTrigger.WhileTrue(autos::VisionDrive(&m_visionSubsystem, &m_driveSubsystem));

  //PID elevator subsystem command
  m_driverController.A().OnTrue(m_elevatorSubsystem.MoveToLevelCommand(ElevatorConstants::level1Goal));
  m_driverController.X().OnTrue(m_elevatorSubsystem.MoveToLevelCommand(ElevatorConstants::level2Goal));
  m_driverController.Y().OnTrue(m_elevatorSubsystem.MoveToLevelCommand(ElevatorConstants::level3Goal));
  m_driverController.B().OnTrue(m_elevatorSubsystem.MoveToLevelCommand(ElevatorConstants::level4Goal));

  m_driverController.LeftTrigger().WhileTrue(m_elevatorSubsystem.MoveUpCommand());
  m_driverController.RightTrigger().WhileTrue(m_elevatorSubsystem.MoveDownCommand());

}

void RobotContainer::Configure2024Bindings() {
  // Configure your trigger bindings here
  
  //drivetrain commands
  m_driverController.B().WhileTrue(m_drive.StopCommand()); 

  //intake commands
  m_driverController.LeftBumper().WhileTrue(m_intakeSubsystem.CollectCommand());
  m_driverController.RightBumper().WhileTrue(m_intakeSubsystem.EjectCommand());
  //m_driverController.B().WhileTrue(m_intakeSubsystem.RetractCommand());
  m_driverController.X().WhileTrue(m_intakeSubsystem.ExtendCommand());

  //shootercommands
  m_driverController.Y().WhileTrue(m_shooterSubsystem.ShooterSpeakerCommand());
  m_driverController.A().WhileTrue(m_shooterSubsystem.ShooterAmpCommand());
  //m_driverController.B().WhileTrue(m_shooterSubsystem.ShooterSpeakerAmpCommand());
 

  m_driverController.LeftTrigger().OnTrue(m_intakeAndShootSubsystem.PerformIntakeAndShootCommand()); 

  //left climb commands
  m_joystick.Button(leftUpButton).OnTrue(std::move(m_leftClimbSubsystem.LeftClimbUpCommand()).Repeatedly().WithTimeout(1.5_s));
  m_joystick.Button(leftDownButton).OnTrue(std::move(m_leftClimbSubsystem.LeftClimbDownCommand()).Repeatedly().WithTimeout(1.5_s));

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() 
{
  // An example command will be run in autonomous
  //return autos::ExampleAuto(&m_subsystem);

    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return PathPlannerAuto("Test Auto").ToPtr();

}


frc2::CommandPtr RobotContainer::GetTestCommand() {
  // An example command will be run in autonomous
  return m_drive.SmartDashboardOutputCommand();
}

