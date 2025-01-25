#include "commands/ElevatorAndArmCommand.h"

ElevatorAndArmCommand::ElevatorAndArmCommand(ElevatorAndArmSubsystem* elevatorAndArmSubsystem)
    : m_elevatorAndArmSubsystem{elevatorAndArmSubsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_elevatorAndArmSubsystem);
}
