// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlignDriveCommand.h"

#include <frc2/command/Commands.h>

#include "Constants.h"

frc2::CommandPtr alignDrive::AlignCommand(DriveSubsystem* drive, Direction direction) {
  {
    // if direction etc.

    frc2::CommandPtr controllerCommand = 
    frc2::SwerveControllerCommand<4>(
          drive->CalculateTrajectory(1_m, 1_m, 0_tr), [drive]() {return drive->getPose(); },

          drive->kinematics,

          frc::PIDController{DriveConstants::kPXController, 0, 0}, //Issue 76 Change Constants
          frc::PIDController{DriveConstants::kPYController, 0, 0},
          drive->thetaController,

          [drive](auto states) {drive->SetModuleStates(states); },

          {drive})
          .ToPtr();

    return frc2::cmd::Sequence(
        std::move(controllerCommand),
        frc2::InstantCommand(
            [drive] {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false); }, {}
            )
            .ToPtr());
  }
}

