// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/ArmSubsystem.h"
#include "subsystems/VisionSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ElevatorAndArmSubsystem.h"

namespace autos {
/**
 * Example static factory for an autonomous command.
 */
frc2::CommandPtr ExampleAuto(ArmSubsystem* subsystem);
frc2::CommandPtr Auto_1(VisionSubsystem* visionSubsystem, ElevatorAndArmSubsystem* elevatorAndArmSubsystem);
}  // namespace autos
