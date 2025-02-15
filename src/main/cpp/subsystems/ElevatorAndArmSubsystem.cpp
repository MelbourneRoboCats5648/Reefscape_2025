#include "subsystems/ElevatorAndArmSubsystem.h"

ElevatorAndArmSubsystem::ElevatorAndArmSubsystem(ElevatorSubsystem& elevatorSub, ArmSubsystem& armSub) 
  : m_elevatorSubsystem(elevatorSub),
    m_armSubsystem(armSub)
{

}
frc2::CommandPtr ElevatorAndArmSubsystem::MoveUp() {
  return m_elevatorSubsystem.MoveUpCommand()
  .AndThen(m_armSubsystem.MoveUpCommand())
   .FinallyDo([this]{
    m_armSubsystem.StopMotor(); });
}

frc2::CommandPtr ElevatorAndArmSubsystem::MoveDown() {
  return m_elevatorSubsystem.MoveDownCommand()
  .AndThen(m_armSubsystem.MoveDownCommand())
   .FinallyDo([this]{
    m_armSubsystem.StopMotor(); });
}
 
frc2::CommandPtr ElevatorAndArmSubsystem::MoveToLevel(Level level) {
  
  switch(level)
  {
    case (Level::L1):
    {
      return ReturnCommandWithCorrectLevel(ElevatorConstants::level1Goal, ArmConstants::level1Goal);
    }
    case (Level::L2):
    {
      return ReturnCommandWithCorrectLevel(ElevatorConstants::level2Goal, ArmConstants::level2Goal);
    }
    case (Level::L3):
    {
      return ReturnCommandWithCorrectLevel(ElevatorConstants::level3Goal, ArmConstants::level3Goal);
    }
    case (Level::L4):
    {
      return ReturnCommandWithCorrectLevel(ElevatorConstants::level4Goal, ArmConstants::level4Goal);
    }
    default:
    {
      // do nothing
    }
  }

}

frc2::CommandPtr ElevatorAndArmSubsystem::ReturnCommandWithCorrectLevel(units::turn_t elevGoal, units::turn_t armGoal)
{
  return m_elevatorSubsystem.MoveToHeightCommand(elevGoal)
            .AndThen(m_armSubsystem.MoveToAngleCommand(armGoal))
            .FinallyDo([this]{m_armSubsystem.StopMotor(); });
}

void ElevatorAndArmSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ElevatorAndArmSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
