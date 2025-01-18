#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ShooterSubsystem.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ShooterCommand
    : public frc2::CommandHelper<frc2::Command, ShooterCommand> {
 public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param ShooterSubsystem The subsystem used by this command.
   */
  explicit ShooterCommand(ShooterSubsystem* shooterSubsystem);

 private:
  ShooterSubsystem* m_shooterSubsystem;
};
