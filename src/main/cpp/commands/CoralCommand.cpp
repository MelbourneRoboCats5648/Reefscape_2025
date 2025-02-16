// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CoralCommand.h"

#include <frc2/command/Commands.h>

#include "commands/ExampleCommand.h"

#include "Constants.h"

using namespace DriveConstants;
using namespace ArmConstants;
using namespace ElevatorConstants;


frc2::CommandPtr coralCommands::CollectCoral(ElevatorAndArmSubsystem* elevatorAndArm, DriveSubsystem* drive) {
  return frc2::cmd::Run([elevatorAndArm] //align using april tag with coral station, may or may not be necessary
  {
    elevatorAndArm->ArmMoveToAngle(aLevel0Goal);
  }
  ).AndThen([elevatorAndArm]
  {
    elevatorAndArm->ElevatorMoveToHeight(eLevel0Goal);
  }
  );
}

frc2::CommandPtr coralCommands::PlaceOnReef(ElevatorAndArmSubsystem* elevatorAndArm, DriveSubsystem* drive) {
  return frc2::cmd::Run([elevatorAndArm, drive]
  {

  }
  );
}

