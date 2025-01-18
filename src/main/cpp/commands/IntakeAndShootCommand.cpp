#include "commands/IntakeAndShootCommand.h"

IntakeAndShootCommand::IntakeAndShootCommand(IntakeAndShootSubsystem* intakeAndShootSubsystem)
    : m_intakeAndShootSubsystem{intakeAndShootSubsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_intakeAndShootSubsystem);
}
