#include "subsystems/ElevatorAndArmSubsystem.h"

using namespace ElevatorConstants;
using namespace ArmConstants;

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

frc2::CommandPtr ElevatorAndArmSubsystem::ElevatorMoveUp() {
  return m_elevatorSubsystem.MoveUpCommand();
 }

frc2::CommandPtr ElevatorAndArmSubsystem::ElevatorMoveDown() {
  return m_elevatorSubsystem.MoveDownCommand();
}

frc2::CommandPtr ElevatorAndArmSubsystem::ArmMoveUp() {
  return m_armSubsystem.MoveUpCommand()
   .FinallyDo([this]{
    m_armSubsystem.StopMotor(); });
}

frc2::CommandPtr ElevatorAndArmSubsystem::ArmMoveDown() {
  return m_armSubsystem.MoveDownCommand()
   .FinallyDo([this]{
    m_armSubsystem.StopMotor(); });
}

frc2::CommandPtr ElevatorAndArmSubsystem::ArmMoveToAngle(units::turn_t armGoal) {
  return m_armSubsystem.MoveToAngleCommand(armGoal);
}


frc2::CommandPtr ElevatorAndArmSubsystem::ElevatorMoveToHeight(units::meter_t elevGoal) {
    return m_elevatorSubsystem.MoveToHeightCommand(elevGoal);
            
}

 units::meter_t ElevatorAndArmSubsystem::ElevatorGetHeight() {
    return m_elevatorSubsystem.GetElevatorHeight();
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

//For collecting coral, if elevator height is not high enough, 
//arm will not be able to rotate and then elevator move down to collect coral, 
//so in this case the arm must move upwards
frc2::CommandPtr ElevatorAndArmSubsystem::CollectCoral(){
  if(ElevatorGetHeight() >= kElevatorMinHeightCollect)
  {
      return (ArmMoveToAngle(aLevel0Goal))
              .AndThen(ElevatorMoveToHeight(eLevel0Goal));
  }
  else
  {
      //issue 70 make this a new goal
      return ElevatorMoveToHeight(eLevel2Goal)
              .AndThen(ArmMoveToAngle(aLevel0Goal))
              .AndThen(ElevatorMoveToHeight(eLevel0Goal));
  }
}

frc2::CommandPtr ElevatorAndArmSubsystem::PlaceCoral(){
  return m_elevatorSubsystem.MoveUpBy(kElevatorPlaceCoral)
          .AlongWith(m_armSubsystem.RotateBy(kArmPlaceCoral));
}


void ElevatorAndArmSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ElevatorAndArmSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
