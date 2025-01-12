#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/RightClimbSubsystem.h"

/**
 * An right climb command that uses an right climb subsystem.
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RightClimbCommand
    : public frc2::CommandHelper<frc2::Command, RightClimbCommand> {
 public:
  /**
   * Creates a new RightClimbCommand.
   *
   * @param RightClimbSubsystem The subsystem used by this command.
   */
  explicit RightClimbCommand(RightClimbSubsystem* rightClimbSubsystem);

 private:
  RightClimbSubsystem* m_rightClimbSubsystem;
};
