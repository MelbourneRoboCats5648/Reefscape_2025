#include "commands/ElevatorCommand.h"

ElevatorCommand::ElevatorCommand(ElevatorSubsystem* elevatorSubsystem)
    : m_elevatorSubsystem{elevatorSubsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_elevatorSubsystem);
}
