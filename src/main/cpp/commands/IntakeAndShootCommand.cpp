#include "commands/IntakeAndShootCommand.h"

IntakeAndShootCommand::IntakeAndShootCommand(IntakeAndShootSubsystem* IntakeAndShootSubsystem)
    : m_intakeAndShootSubsystem{IntakeAndShootSubsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_intakeAndShootSubsystem);
}
