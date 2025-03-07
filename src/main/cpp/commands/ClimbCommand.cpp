#include "commands/ClimbCommand.h"

ClimbCommand::ClimbCommand(ClimbSubsystem* climbSubsystem)
    : m_climbSubsystem{climbSubsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_climbSubsystem);
}
