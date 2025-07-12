// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"


#include <frc2/command/button/Trigger.h>

//copied from frc example code
#include <utility>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "commands/Autos.h"

#include <frc/filter/SlewRateLimiter.h>

using namespace DriveConstants;

double RobotContainer::GetMechLeftY() {
  return frc::ApplyDeadband(m_mechController.GetLeftY(), kMechDeadband);
}

double RobotContainer::GetMechRightY() {
  return frc::ApplyDeadband(m_mechController.GetRightY(), kMechDeadband);
}


double RobotContainer::ScaleJoystickInput(double input) {
  /* no scaling */
  // return input;

  /* square scaling */
  double sign = (input < 0.0) ? -1.0 : 1.0; // since we'll be losing the sign when we square
  return sign * input * input;

  /* cube scaling */
  // return input * input * input;
}

RobotContainer::RobotContainer()
   :m_drive(),
    m_elevatorSubsystem(),
    m_armSubsystem(),
    m_elevatorAndArmSubsystem(m_elevatorSubsystem, m_armSubsystem),
    m_visionSubsystem(m_drive)
{
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();

  m_drive.SetDefaultCommand(frc2::RunCommand(
    [this] {
      units::velocity::meters_per_second_t xspeed = -m_drive.m_xLimiter.Calculate(ScaleJoystickInput(frc::ApplyDeadband(m_driverController.GetLeftY(), kDriverDeadband)) * kMaxSpeed);
      units::velocity::meters_per_second_t yspeed = -m_drive.m_yLimiter.Calculate(ScaleJoystickInput(frc::ApplyDeadband(m_driverController.GetLeftX(), kDriverDeadband)) * kMaxSpeed);
      units::angular_velocity::radians_per_second_t rotspeed = -m_drive.m_rotLimiter.Calculate(ScaleJoystickInput(frc::ApplyDeadband(m_driverController.GetRightX(), kDriverDeadband)) * kMaxAngularSpeed);
      
      frc::SmartDashboard::PutNumber("xspeed", xspeed.value());
      frc::SmartDashboard::PutNumber("yspeed", yspeed.value());
      frc::SmartDashboard::PutNumber("rotspeed", rotspeed.value());

      m_drive.Drive(
          // Multiply by max speed to map the joystick unitless inputs to
          // actual units. This will map the [-1, 1] to [max speed backwards,
          // max speed forwards], converting them to actual units.
          xspeed, yspeed, rotspeed);
    },
    {&m_drive}));

  /* elevator velocity control override */
  frc2::Trigger elevatorOverrideTrigger([this] { return std::abs(GetMechLeftY()) > General::kFloatTolerance; });
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
  frc2::Trigger armOverrideTrigger([this] { return std::abs(GetMechRightY()) > General::kFloatTolerance; });
  armOverrideTrigger.WhileTrue(frc2::RunCommand([this] {
    m_armSubsystem.VelocityControl(-ScaleJoystickInput(GetMechRightY()) * ArmConstants::kManualMaxVelocity); // so that up makes the arm go up - TODO: check if we want to scale joystick input here
  }, { &m_armSubsystem }).ToPtr());
}


void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // mech controller bindings
  m_mechController.A().OnTrue(m_elevatorAndArmSubsystem.CollectCoral());
  m_mechController.X().OnTrue(m_elevatorAndArmSubsystem.MoveToLevel(Level::L1));
  m_mechController.Y().OnTrue(m_elevatorAndArmSubsystem.MoveToLevel(Level::L2));
  m_mechController.B().OnTrue(m_elevatorAndArmSubsystem.MoveToLevel(Level::L3));
  m_mechController.RightTrigger().OnTrue(m_elevatorAndArmSubsystem.PlaceCoral());
  m_mechController.LeftTrigger().OnTrue(m_elevatorAndArmSubsystem.DefaultPositionCommand());

  // drive controller bindings
  m_driverController.A().WhileTrue(m_climbSubsystem.MoveDownCommand());
  m_driverController.B().WhileTrue(m_climbSubsystem.MoveClimbCommand(-1.0));
  m_driverController.X().OnTrue(m_climbSubsystem.MoveToAngleCommand(ClimbConstants::extendGoal));
  m_driverController.Y().OnTrue(m_climbSubsystem.MoveToAngleCommand(ClimbConstants::retractGoal));

  m_driverController.RightTrigger().OnTrue(m_drive.ToggleFieldRelativeCommand());


  m_driverController.LeftBumper().WhileTrue(m_visionSubsystem.MoveToTarget(ReefPosition::Left));
  m_driverController.RightBumper().WhileTrue(m_visionSubsystem.MoveToTarget(ReefPosition::Right));

  m_mechController.LeftBumper().WhileTrue(m_visionSubsystem.MoveToTarget(ReefPosition::Left));
  m_mechController.RightBumper().WhileTrue(m_visionSubsystem.MoveToTarget(ReefPosition::Right));
      
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

