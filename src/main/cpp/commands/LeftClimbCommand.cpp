#include "commands/LeftClimbCommand.h"

LeftClimbCommand::LeftClimbCommand(LeftClimbSubsystem* leftClimbSubsystem)
    : m_leftClimbSubsystem{leftClimbSubsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_leftClimbSubsystem);
}
