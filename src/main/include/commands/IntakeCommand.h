#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class IntakeCommand
    : public frc2::CommandHelper<frc2::Command, IntakeCommand> {
 public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param IntakeSubsystem The subsystem used by this command.
   */
  explicit IntakeCommand(IntakeSubsystem* intakeSubsystem);

 private:
  IntakeSubsystem* m_intakeSubsystem;
};
