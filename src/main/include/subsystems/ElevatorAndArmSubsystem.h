#pragma once

#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/ArmSubsystem.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

class ElevatorAndArmSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorAndArmSubsystem(ElevatorSubsystem& elevatorSub,
    ArmSubsystem& armSub);

  frc2::CommandPtr Level1();
  frc2::CommandPtr Level2();
  frc2::CommandPtr Level3();
  frc2::CommandPtr MoveDown();

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ElevatorSubsystem& m_elevatorSubsystem;
  ArmSubsystem& m_armSubsystem;

};

