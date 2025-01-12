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

  static frc::SlewRateLimiter<units::scalar> xLimiter{2 / 1_s};
  static frc::SlewRateLimiter<units::scalar> yLimiter{2 / 1_s};
  static frc::SlewRateLimiter<units::scalar> rotLimiter{2 / 1_s};

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();

    m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(
            // Multiply by max speed to map the joystick unitless inputs to
            // actual units. This will map the [-1, 1] to [max speed backwards,
            // max speed forwards], converting them to actual units.
            xLimiter.Calculate(m_driverController.GetLeftY() * DriveConstants::kMaxSpeed),
            yLimiter.Calculate(m_driverController.GetLeftX() * DriveConstants::kMaxSpeed),
            rotLimiter.Calculate(m_driverController.GetRightX() * DriveConstants::kMaxAngularSpeed),
            false
            
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
  m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
  
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
