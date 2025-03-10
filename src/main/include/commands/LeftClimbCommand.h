#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/LeftClimbSubsystem.h"

/**
 * An LeftClimb command that uses an LeftClimb subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class LeftClimbCommand
    : public frc2::CommandHelper<frc2::Command, LeftClimbCommand> {
 public:
  /**
   * Creates a new LeftClimbCommand.
   *
   * @param LeftClimbSubsystem The subsystem used by this command.
   */
  explicit LeftClimbCommand(LeftClimbSubsystem* leftClimbSubsystem);

 private:
  LeftClimbSubsystem* m_leftClimbSubsystem;
};
