// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>
#include <frc2/command/WaitCommand.h>

#include "commands/ArmCommand.h"


frc2::CommandPtr autos::ExampleAuto(ArmSubsystem* subsystem) {
  return frc2::cmd::Sequence(subsystem->MoveUpCommand(),
                             ArmCommand(subsystem).ToPtr());
}

frc2::CommandPtr autos::Auto_1(VisionSubsystem* visionSubsystem, ElevatorAndArmSubsystem* elevatorAndArmSubsystem) {
  frc::Pose2d targetPose = visionSubsystem->GetLeftPose(visionSubsystem->GetTagPose(22)); // 6 for red, 22 for blue

  return visionSubsystem->MoveToPose(targetPose)
  .AlongWith(elevatorAndArmSubsystem->MoveToLevel(Level::L3).WithTimeout(4.0_s))
  // .AndThen(elevatorAndArmSubsystem->PlaceCoral())
  .AndThen(visionSubsystem->MoveToPose(targetPose.TransformBy(frc::Transform2d{
    frc::Translation2d{0.300_m, 0.000_m},
    frc::Rotation2d{0_rad}
  })).WithTimeout(1.0_s))
  .AndThen(elevatorAndArmSubsystem->DefaultPositionCommand());
}


