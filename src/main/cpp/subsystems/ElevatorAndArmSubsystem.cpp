#include "subsystems/ElevatorAndArmSubsystem.h"

ElevatorAndArmSubsystem::ElevatorAndArmSubsystem(ElevatorSubsystem& elevatorSub, ArmSubsystem& armSub) 
  : m_elevatorSubsystem(elevatorSub),
    m_armSubsystem(armSub)
{

}
frc2::CommandPtr ElevatorAndArmSubsystem::MoveUp() {
  return m_elevatorSubsystem.MoveUpCommand()
  .AndThen(m_armSubsystem.MoveArmUpCommand())
   .FinallyDo([this]{
    m_armSubsystem.StopMotor(); });
}

frc2::CommandPtr ElevatorAndArmSubsystem::MoveDown() {
  return m_elevatorSubsystem.MoveDownCommand()
  .AndThen(m_armSubsystem.MoveArmDownCommand())
   .FinallyDo([this]{
    m_armSubsystem.StopMotor(); });
}

frc2::CommandPtr ElevatorAndArmSubsystem::MoveToLevel() {
  return m_elevatorSubsystem.MoveToLevelCommand()
  .AndThen(m_armSubsystem.MoveArmToLevelCommand())
   .FinallyDo([this]{
    m_armSubsystem.StopMotor(); });
}

void ElevatorAndArmSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ElevatorAndArmSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
