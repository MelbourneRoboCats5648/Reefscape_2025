// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>

#include "commands/ExampleCommand.h"

#include "Constants.h"

frc2::CommandPtr autos::ExampleAuto(ExampleSubsystem* subsystem) {
  return frc2::cmd::Sequence(subsystem->ExampleMethodCommand(),
                             ExampleCommand(subsystem).ToPtr());
}

using namespace DriveConstants;



frc2::CommandPtr autos::VisionDrive(VisionSubsystem* visionSubsystem, DriveSubsystem* driveSubsystem) {
  return frc2::cmd::Run([visionSubsystem, driveSubsystem]
  {
    units::meters_per_second_t ForwardSpeed = visionSubsystem->GetTargetingForwardSpeedReef() * kMaxSpeed;
    units::radians_per_second_t AngularVelocity = visionSubsystem->GetTargetingAngularVelocityReef() * kMaxAngularSpeed;
    auto visionX_Speed = ForwardSpeed;
    auto visionRot = AngularVelocity;

    auto visionY_Speed = 0_mps;
    driveSubsystem->Drive(visionX_Speed, visionY_Speed, visionRot, false);
  }
  ).FinallyDo([driveSubsystem]
  {  
    driveSubsystem->StopAllModules();
  });
}

