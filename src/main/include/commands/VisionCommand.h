#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/commands.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>

#include "subsystems/VisionSubsystem.h"
#include "subsystems/DriveSubsystem.h"

namespace cmd {
/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class VisionCommand
    : public frc2::CommandHelper<frc2::Command, VisionCommand> {
 public:
  /**
   * Creates a new VisionCommand.
   *
   * @param visionSubsystem The subsystem used by this command.
   * @ param driveSubsystem The subsystem used by this command.
   */
  VisionCommand(VisionSubsystem* visionSubsystem, DriveSubsystem* driveSubsystem);
  
    double m_targettingAngularVelocity();
    double m_targettingForwardSpeed(); 

  frc2::CommandPtr AimRobotToTargetReef();

 private:
  VisionSubsystem* m_visionSubsystem;
  DriveSubsystem* m_driveSubsystem;
};

}//namespace cmd