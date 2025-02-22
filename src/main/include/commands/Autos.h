// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/commands.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>

#include "subsystems/VisionSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ExampleSubsystem.h"
#include "Constants.h"

namespace autos {
/**
 * Example static factory for an autonomous command.
 */
frc2::CommandPtr ExampleAuto(ExampleSubsystem* subsystem);

frc2::CommandPtr VisionDrive(VisionSubsystem* visionSubsystem,
                             DriveSubsystem* driveSubsystem);
}  // namespace autos

