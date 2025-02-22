// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/ElevatorAndArmSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "Constants.h"
#include "Autos.h"

namespace coralCommands {
/**
 * Example static factory for an autonomous command.
 */
frc2::CommandPtr PlaceOnReef(ElevatorAndArmSubsystem* elevatorAndArm, DriveSubsystem* drive, VisionSubsystem* vision, Level level, Direction direction);

}  // namespace autos
