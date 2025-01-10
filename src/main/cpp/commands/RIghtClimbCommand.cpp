#include "commands/RightClimbCommand.h"

RightClimbCommand::RightClimbCommand(RightClimbSubsystem* rightClimbSubsystem)
    : m_rightClimbSubsystem{rightClimbSubsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_rightClimbSubsystem);
}
