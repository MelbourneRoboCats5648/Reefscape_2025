#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/RightClimbSubsystem.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RightClimbCommand
    : public frc2::CommandHelper<frc2::Command, RightClimbCommand> {
 public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param RightClimbSubsystem The subsystem used by this command.
   */
  explicit RightClimbCommand(RightClimbSubsystem* rightClimbSubsystem);

 private:
  RightClimbSubsystem* m_rightClimbSubsystem;
};
