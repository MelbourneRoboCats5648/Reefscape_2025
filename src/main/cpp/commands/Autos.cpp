// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>

#include "commands/ArmCommand.h"


frc2::CommandPtr autos::ExampleAuto(ArmSubsystem* subsystem) {
  return frc2::cmd::Sequence(subsystem->MoveUpCommand(),
                             ArmCommand(subsystem).ToPtr());
}

frc2::CommandPtr autos::Auto_1(VisionSubsystem* visionSubsystem, ElevatorAndArmSubsystem* elevatorAndArmSubsystem) {
  
  frc::Pose2d targetPose = visionSubsystem->GetLeftPose(visionSubsystem->GetTagPose(17));

  return visionSubsystem->MoveToPose(targetPose)
  .AndThen(elevatorAndArmSubsystem->MoveToLevel(Level::L3))
  .AndThen(elevatorAndArmSubsystem->PlaceCoral());
}


