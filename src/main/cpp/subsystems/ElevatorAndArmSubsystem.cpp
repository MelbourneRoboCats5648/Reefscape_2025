#include "subsystems/ElevatorAndArmSubsystem.h"

ElevatorAndArmSubsystem::ElevatorAndArmSubsystem(ElevatorSubsystem& elevatorSub, ArmSubsystem& armSub) 
  : m_elevatorSubsystem(elevatorSub),
    m_armSubsystem(armSub)
{

}
frc2::CommandPtr ElevatorAndArmSubsystem::Level1() {
  return m_elevatorSubsystem.MoveUpToL1Command()
  .AndThen(m_armSubsystem.MoveArmToL1Command())
   .FinallyDo([this]{
    m_elevatorSubsystem.StopMotors();
    m_armSubsystem.StopMotor(); });
}

frc2::CommandPtr ElevatorAndArmSubsystem::Level2() {
  return m_elevatorSubsystem.MoveUpToL2Command()
  .AndThen(m_armSubsystem.MoveArmToL2Command())
   .FinallyDo([this]{
    m_elevatorSubsystem.StopMotors();
    m_armSubsystem.StopMotor(); });
}

frc2::CommandPtr ElevatorAndArmSubsystem::Level3() {
  return m_elevatorSubsystem.MoveUpToL3Command()
  .AndThen(m_armSubsystem.MoveArmToL3Command())
   .FinallyDo([this]{
    m_elevatorSubsystem.StopMotors();
    m_armSubsystem.StopMotor(); });
}

frc2::CommandPtr ElevatorAndArmSubsystem::MoveDown() {
  return m_elevatorSubsystem.MoveDownCommand()
  .AndThen(m_armSubsystem.MoveArmDownCommand())
   .FinallyDo([this]{
    m_elevatorSubsystem.StopMotors();
    m_armSubsystem.StopMotor(); });
}

void ElevatorAndArmSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ElevatorAndArmSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
