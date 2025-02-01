#include "commands/IntakeCommand.h"

IntakeCommand::IntakeCommand(IntakeSubsystem* intakeSubsystem)
    : m_intakeSubsystem{intakeSubsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_intakeSubsystem);
}
