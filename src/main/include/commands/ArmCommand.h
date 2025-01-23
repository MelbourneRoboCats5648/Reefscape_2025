#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ArmSubsystem.h"

/**
 * An arm command that uses an arm subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ArmCommand
    : public frc2::CommandHelper<frc2::Command, ArmCommand> {
 public:
  /**
   * Creates a new ElevatorCommand.
   *
   * @param ArmSubsystem The subsystem used by this command.
   */
  explicit ArmCommand(ArmSubsystem* armSubsystem);

 private:
  ArmSubsystem* m_armSubsystem;
};
