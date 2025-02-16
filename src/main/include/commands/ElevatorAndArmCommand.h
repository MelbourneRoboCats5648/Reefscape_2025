#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ElevatorAndArmSubsystem.h"


class ElevatorAndArmCommand
    : public frc2::CommandHelper<frc2::Command, ElevatorAndArmCommand> {
 public:
  /**
   * Creates a new ElevatorAndArmCommand.
   * 
   * @param elevatorAndArmSubsystem The subsystem used by this command.
   */
  explicit ElevatorAndArmCommand(ElevatorAndArmSubsystem* elevatorAndArmSubsystem);

 private:
  ElevatorAndArmSubsystem* m_elevatorAndArmSubsystem;
};