#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeAndShootSubsystem.h"


class IntakeAndShootCommand
    : public frc2::CommandHelper<frc2::Command, IntakeAndShootCommand> {
 public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param IntakeAndShootSubsystem The subsystem used by this command.
   */
  explicit IntakeAndShootCommand(IntakeAndShootSubsystem* intakeAndShootSubsystem);

 private:
  IntakeAndShootSubsystem* m_intakeAndShootSubsystem;
};
