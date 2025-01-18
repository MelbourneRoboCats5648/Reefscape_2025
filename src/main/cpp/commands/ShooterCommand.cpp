#include "commands/ShooterCommand.h"

ShooterCommand::ShooterCommand(ShooterSubsystem* shooterSubsystem)
    : m_shooterSubsystem{shooterSubsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_shooterSubsystem);
}
