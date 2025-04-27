#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ClimbSubsystem.h"

/**
 * A climb command that uses an climb subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ClimbCommand
    : public frc2::CommandHelper<frc2::Command, ClimbCommand> {
 public:
  /**
   * Creates a new ClimbCommand.
   *
   * @param ClimbSubsystem The subsystem used by this command.
   */
  explicit ClimbCommand(ClimbSubsystem* climbSubsystem);

 private:
  ClimbSubsystem* m_climbSubsystem;
};
