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

frc2::CommandPtr ElevatorAndArmSubsystem::ArmMoveToAngle(units::turn_t armGoal) {
  return m_armSubsystem.MoveToAngleCommand(armGoal);
}


frc2::CommandPtr ElevatorAndArmSubsystem::ElevatorMoveToHeight(units::meter_t elevGoal) {
    return m_elevatorSubsystem.MoveToHeightCommand(elevGoal);
            
}

frc2::CommandPtr ElevatorAndArmSubsystem::MoveToLevel(Level level) {
  
  units::meter_t elevGoal;
  units::turn_t armGoal;

  switch(level)
  {
    case (Level::L0):
    {
      elevGoal = ElevatorConstants::eLevel0Goal;
      armGoal = ArmConstants::aLevel0Goal;
      break;
    }
    case (Level::L1):
    {
      elevGoal = ElevatorConstants::eLevel1Goal;
      armGoal = ArmConstants::aLevel1Goal;
      break;
    }
    case (Level::L2):
    {
      elevGoal = ElevatorConstants::eLevel2Goal;
      armGoal = ArmConstants::aLevel2Goal;
      break;
    }
    case (Level::L3):
    {
      elevGoal = ElevatorConstants::eLevel3Goal;
      armGoal = ArmConstants::aLevel3Goal;
      break;
    }
    case (Level::L4):
    {
      elevGoal = ElevatorConstants::eLevel4Goal;
      armGoal = ArmConstants::aLevel4Goal;
      break;
    }
    default:
    {
      // do nothing
    }
  }

  return m_elevatorSubsystem.MoveToHeightCommand(elevGoal)
            .AlongWith(m_armSubsystem.MoveToAngleCommand(armGoal));
}

void ElevatorAndArmSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ElevatorAndArmSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
