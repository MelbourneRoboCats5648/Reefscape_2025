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

#include <frc/filter/SlewRateLimiter.h>
#include <frc/Joystick.h>

using namespace DriveConstants;

RobotContainer::RobotContainer()
   :m_elevatorSubsystem(),
    m_armSubsystem(),
    m_elevatorAndArmSubsystem(m_elevatorSubsystem, m_armSubsystem)
{

  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  switch(General::KBuildSeason) {
    case (BuildSeason::Reefscape): {
      ConfigureBindings();
      break;
    }    
    case (BuildSeason::Crescendo): {
      Configure2024Bindings();
      break;
    }
  }

  switch(General::KTestLevel) {
    case (TestLevel::NONE): {
      break;
    }
    case (TestLevel::ARM): {
      // m_elevatorAndArmSubsystem.SetDefaultCommand(frc2::RunCommand(
      //   [this] {
      //     double speed = frc::ApplyDeadband(m_driverController.GetLeftY(), kDeadband);
      //     m_elevatorAndArmSubsystem.MoveArm(speed);
      //   },
      //   {&m_elevatorAndArmSubsystem}));

        break;
    }
    case (TestLevel::ELEVATOR): {
      // m_elevatorAndArmSubsystem.SetDefaultCommand(frc2::RunCommand(
      //   [this] {
      //     double speed = frc::ApplyDeadband(m_driverController.GetLeftY(), kDeadband);
      //     m_elevatorAndArmSubsystem.MoveSecondStage(speed);
      //   },
      //   {&m_elevatorAndArmSubsystem}));

        break;
    }
    case (TestLevel::DRIVE): {
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

        break;
    }

  }

  





}




void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
    //drivetrain commands
  //m_driverController.B().WhileTrue(m_drive.StopCommand());

  // elevator move up/down
  // m_driverController.Y().WhileTrue(m_elevatorSubsystem.m_firstStage.MoveUpCommand());
  // m_driverController.X().WhileTrue(m_elevatorSubsystem.m_firstStage.MoveDownCommand());
  // m_driverController.B().WhileTrue(m_elevatorSubsystem.m_secondStage.MoveUpCommand());
  // m_driverController.A().WhileTrue(m_elevatorSubsystem.m_secondStage.MoveDownCommand());

  // elevator position ctrl
  // m_driverController.Y().WhileTrue(m_elevatorSubsystem.m_firstStage.MoveToHeightCommand(ElevatorConstants::kMaxFirstStageHeight));
  // m_driverController.X().WhileTrue(m_elevatorSubsystem.m_firstStage.MoveToHeightCommand(ElevatorConstants::kInitFirstStageHeight + 0.05_m));
  // m_driverController.B().WhileTrue(m_elevatorSubsystem.m_secondStage.MoveToHeightCommand(ElevatorConstants::kMaxSecondStageHeight));
  // m_driverController.A().WhileTrue(m_elevatorSubsystem.m_secondStage.MoveToHeightCommand(ElevatorConstants::kInitSecondStageHeight + 0.05_m));

  // //PID elevator subsystem command
  // m_driverController.A().OnTrue(m_elevatorAndArmSubsystem.MoveToLevel(Level::L1));
  // m_driverController.X().OnTrue(m_elevatorAndArmSubsystem.MoveToLevel(Level::L2));
  // m_driverController.Y().OnTrue(m_elevatorAndArmSubsystem.MoveToLevel(Level::L3));
  // m_driverController.B().OnTrue(m_elevatorAndArmSubsystem.MoveToLevel(Level::L4));

  // issue 102 - testing arm goal command
  // m_driverController.A().OnTrue(m_elevatorAndArmSubsystem.ArmMoveToAngle(units::turn_t(0_tr)));
  // m_driverController.X().OnTrue(m_elevatorAndArmSubsystem.ArmMoveToAngle(units::turn_t(-0.2_tr)));
  // m_driverController.Y().OnTrue(m_elevatorAndArmSubsystem.ArmMoveToAngle(units::turn_t(-0.3_tr)));
  // m_driverController.B().OnTrue(m_elevatorAndArmSubsystem.ArmMoveToAngle(units::turn_t(0.15_tr)));


  // // Move to height
  m_mechController.A().OnTrue(m_elevatorAndArmSubsystem.ElevatorMoveToHeight(ElevatorConstants::retractSoftLimitSecondStage));
  m_mechController.B().OnTrue(m_elevatorAndArmSubsystem.ElevatorMoveToHeight(ElevatorConstants::kMaxSecondStageHeight));
  m_mechController.X().OnTrue(m_elevatorAndArmSubsystem.ElevatorMoveToHeight(ElevatorConstants::kMaxFirstStageHeight));
  m_mechController.Y().OnTrue(m_elevatorAndArmSubsystem.ElevatorMoveToHeight(ElevatorConstants::kMaxFirstStageHeight + ElevatorConstants::kMaxSecondStageHeight));






  //Collect Command
//  m_driverController.LeftBumper().OnTrue(m_elevatorAndArmSubsystem.CollectCoral());

  //Place on Reef Command
//  m_driverController.RightBumper().OnTrue(m_elevatorAndArmSubsystem.PlaceCoral());

  // PID climb subsystem command (temporarily disabling LeftClimbUpCommand and LeftClimbDownCommand to test it)
  //m_joystick.Button(LeftClimbConstants::leftUpButton).OnTrue(m_leftClimbSubsystem.LeftClimbCommand(GoalConstants::m_climbGoalL1));
  //m_joystick.Button(LeftClimbConstants::leftDownButton).OnTrue(m_leftClimbSubsystem.LeftClimbCommand(GoalConstants::m_climbGoalRetract));

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is pressed, cancelling on release.
//  m_driverController.LeftTrigger().WhileTrue(m_elevatorAndArmSubsystem.ElevatorMoveUp());
//  m_driverController.RightTrigger().WhileTrue(m_elevatorAndArmSubsystem.ElevatorMoveDown());
}

void RobotContainer::Configure2024Bindings() {
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_armSubsystem);
}

frc2::CommandPtr RobotContainer::GetTestCommand() {
  // An example command will be run in autonomous
  return m_drive.SmartDashboardOutputCommand();
}

