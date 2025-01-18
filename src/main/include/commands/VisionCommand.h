#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/VisionSubsystem.h"

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
   * Creates a new ExampleCommand.
   *
   * @param ShooterSubsystem The subsystem used by this command.
   */
  explicit VisionCommand(VisionSubsystem* visionSubsystem);

 private:
  VisionSubsystem* m_visionSubsystem;
};
