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

frc2::CommandPtr autos::Auto_1(VisionSubsystem* visionSubsystem, DriveSubsystem* driveSubsystem, ElevatorAndArmSubsystem* elevatorAndArmSubsystem) {
  return frc2::cmd::Run([visionSubsystem, driveSubsystem]
  {
    frc::Pose2d targetPose = visionSubsystem->GetLeftPose(visionSubsystem->GetPose(7));

    frc::Trajectory trajectory = visionSubsystem->CreateTrajectory(targetPose);

    visionSubsystem->Followtrajectory(trajectory);

    //Move to target takes left or right enum, 
    //but it uses the april tag that it can see. 
    //to make it a completely general command, i think we need to pass it an id as well at this point in time.
    //but idk.
  }).AndThen([elevatorAndArmSubsystem]{
    elevatorAndArmSubsystem->MoveToLevel(Level::L3);
  }).AndThen([elevatorAndArmSubsystem]{
    elevatorAndArmSubsystem->PlaceCoral();
  });

}