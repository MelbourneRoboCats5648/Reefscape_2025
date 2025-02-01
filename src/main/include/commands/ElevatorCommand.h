#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ElevatorSubsystem.h"

/**
 * An elevator command that uses an elevator subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ElevatorCommand
    : public frc2::CommandHelper<frc2::Command, ElevatorCommand> {
 public:
  /**
   * Creates a new ElevatorCommand.
   *
   * @param ElevatorSubsystem The subsystem used by this command.
   */
  explicit ElevatorCommand(ElevatorSubsystem* elevatorSubsystem);

 private:
  ElevatorSubsystem* m_elevatorSubsystem;
};
