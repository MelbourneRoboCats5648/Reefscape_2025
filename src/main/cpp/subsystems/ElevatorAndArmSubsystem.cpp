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
  
  units::meter_t elevGoal;
  units::turn_t armGoal;

  switch(level)
  {
    case (Level::L0):
    {
      elevGoal = ElevatorConstants::level0Goal;
      armGoal = ArmConstants::level0Goal;
      break;
    }
    case (Level::L1):
    {
      elevGoal = ElevatorConstants::level1Goal;
      armGoal = ArmConstants::level1Goal;
      break;
    }
    case (Level::L2):
    {
      elevGoal = ElevatorConstants::level2Goal;
      armGoal = ArmConstants::level2Goal;
      break;
    }
    case (Level::L3):
    {
      elevGoal = ElevatorConstants::level3Goal;
      armGoal = ArmConstants::level3Goal;
      break;
    }
    case (Level::L4):
    {
      elevGoal = ElevatorConstants::level4Goal;
      armGoal = ArmConstants::level4Goal;
      break;
    }
    default:
    {
      // do nothing
    }
  }

  return m_elevatorSubsystem.MoveToHeightCommand(elevGoal)
            .AlongWith(m_armSubsystem.MoveToAngleCommand(armGoal))
            .FinallyDo([this]{m_armSubsystem.StopMotor(); });

}

void ElevatorAndArmSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ElevatorAndArmSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
