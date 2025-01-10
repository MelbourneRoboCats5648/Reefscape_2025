#include "commands/ShooterCommand.h"

ShooterCommand::ShooterCommand(ShooterSubsystem* shootersubsystem)
    : m_shootersubsystem{shootersubsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_shootersubsystem);
}
