#include "commands/ArmCommand.h"

ArmCommand::ArmCommand(ArmSubsystem* armSubsystem)
    : m_armSubsystem{armSubsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_armSubsystem);
}
