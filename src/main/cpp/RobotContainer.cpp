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

double RobotContainer::GetMechLeftY() {
  return frc::ApplyDeadband(m_mechController.GetLeftY(), kDeadband);
}

double RobotContainer::GetMechRightY() {
  return frc::ApplyDeadband(m_mechController.GetRightY(), kDeadband);
}

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
              yspeed, xspeed, rotspeed);
        },
        {&m_drive}));

        break;
    }

  }

  /* elevator velocity control override */
  frc2::Trigger elevatorOverrideTrigger([this] { return GetMechLeftY() != 0.0; });
  elevatorOverrideTrigger.WhileTrue(
    /* control 2nd stage */
    frc2::RunCommand([this] {
      m_elevatorSubsystem.m_secondStage.VelocityControl(-GetMechLeftY() * ElevatorConstants::kManualMaxVelocity);
    }, { &m_elevatorSubsystem.m_secondStage }).ToPtr()
    .Until([this] { return m_elevatorSubsystem.m_secondStage.GetHeight() >= ElevatorConstants::kMaxSecondStageHeight; }) // FIXME: this might not work if we attempt to pull 2nd stage down while it's at max height

    /* control 1st stage */
    .AndThen(
      frc2::RunCommand([this] {
        m_elevatorSubsystem.m_secondStage.VelocityControl(-GetMechLeftY() * ElevatorConstants::kManualMaxVelocity);
      }, { &m_elevatorSubsystem.m_firstStage }).ToPtr()
      .Until([this] { return m_elevatorSubsystem.m_firstStage.GetHeight() <= ElevatorConstants::retractSoftLimitFirstStage + ElevatorConstants::kManualRetractLimitTolerance; })
    )
  );

  /* arm velocity control override */
  frc2::Trigger armOverrideTrigger([this] { return GetMechRightY() != 0.0; });
  armOverrideTrigger.WhileTrue(frc2::RunCommand([this] {
    m_armSubsystem.VelocityControl(-GetMechRightY() * ArmConstants::kManualMaxVelocity); // so that up makes the arm go up
  }, { &m_armSubsystem }).ToPtr());
}




void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
    //drivetrain commands
  //m_driverController.B().WhileTrue(m_drive.StopCommand());

  // drive mode
  // m_driverController.LeftTrigger().OnTrue(m_drive.ResetFieldGyroOffsetCommand());
  m_driverController.RightTrigger().OnTrue(m_drive.ToggleFieldRelativeCommand());

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
  m_mechController.A().OnTrue(m_elevatorAndArmSubsystem.CollectCoral());
  m_mechController.X().OnTrue(m_elevatorAndArmSubsystem.MoveToLevel(Level::L1));
  m_mechController.Y().OnTrue(m_elevatorAndArmSubsystem.MoveToLevel(Level::L2));
  m_mechController.B().OnTrue(m_elevatorAndArmSubsystem.MoveToLevel(Level::L3));
  m_mechController.RightTrigger().OnTrue(m_elevatorAndArmSubsystem.PlaceCoral());
  m_mechController.LeftTrigger().OnTrue(m_elevatorAndArmSubsystem.DefaultPositionCommand());

  // issue 102 - testing arm goal command
  // m_driverController.A().OnTrue(m_elevatorAndArmSubsystem.ArmMoveToAngle(units::turn_t(0_tr)));
  // m_driverController.X().OnTrue(m_elevatorAndArmSubsystem.ArmMoveToAngle(units::turn_t(-0.2_tr)));
  // m_driverController.Y().OnTrue(m_elevatorAndArmSubsystem.ArmMoveToAngle(units::turn_t(-0.3_tr)));
  // m_driverController.B().OnTrue(m_elevatorAndArmSubsystem.ArmMoveToAngle(units::turn_t(0.15_tr)));


  // // Move to height
  // m_driverController.A().OnTrue(m_elevatorAndArmSubsystem.ElevatorMoveToHeight(ElevatorConstants::retractSoftLimitSecondStage));
  // m_driverController.B().OnTrue(m_elevatorAndArmSubsystem.ElevatorMoveToHeight(ElevatorConstants::kMaxSecondStageHeight));
  // m_driverController.X().OnTrue(m_elevatorAndArmSubsystem.ElevatorMoveToHeight(ElevatorConstants::kMaxFirstStageHeight));
  // m_driverController.Y().OnTrue(m_elevatorAndArmSubsystem.ElevatorMoveToHeight(ElevatorConstants::kMaxFirstStageHeight + ElevatorConstants::kMaxSecondStageHeight));






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

frc2::CommandPtr RobotContainer::GetInitCommand() {
  return m_elevatorAndArmSubsystem.DefaultPositionCommand();
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return frc2::RunCommand([this] {
    m_drive.Drive(-0.75_mps, 0.0_mps, 0.0_rad_per_s);
  }, {&m_drive}).FinallyDo([this] {
    m_drive.Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s);
  }).WithTimeout(2_s);
}

frc2::CommandPtr RobotContainer::GetTestCommand() {
  // An example command will be run in autonomous
  return m_drive.SmartDashboardOutputCommand();
}

