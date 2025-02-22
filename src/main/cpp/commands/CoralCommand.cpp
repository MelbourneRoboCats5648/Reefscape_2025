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

//align using april tag with coral station, may or may not be necessary
// issue 70 move collect coral command to elevator and arm subsystem and then can stitch in drive stuff here if need be
/*frc2::CommandPtr coralCommands::CollectCoral(ElevatorAndArmSubsystem* elevatorAndArm) {

}
*/

frc2::CommandPtr coralCommands::PlaceOnReef(ElevatorAndArmSubsystem* elevatorAndArm, DriveSubsystem* drive, VisionSubsystem* vision, Level level, Direction direction) {
  return frc2::cmd::Run([elevatorAndArm, drive, vision, level, direction]
  {
    //pass command a level
    //moving elevator and arm to height (then positioning) then move arm and elevator down at the same time (not move to level but move by certain amount, eg 15degrees and down 10cm) 
    elevatorAndArm->MoveToLevel(level);
    autos::VisionDrive(vision, drive);
    //drive->Align(drive, direction);//check joystick button here 
    elevatorAndArm->PlaceCoral();
  }
  );
  }

