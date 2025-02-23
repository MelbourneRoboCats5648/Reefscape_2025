#pragma once

#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/ArmSubsystem.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>


class ElevatorAndArmSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorAndArmSubsystem(ElevatorSubsystem& elevatorSub,
    ArmSubsystem& armSub);

  frc2::CommandPtr MoveUp();
  frc2::CommandPtr MoveDown();
  frc2::CommandPtr ArmMoveToAngle(units::turn_t armGoal);
  frc2::CommandPtr ElevatorMoveToHeight(units::meter_t elevGoal);
  units::meter_t ElevatorGetHeight();
  frc2::CommandPtr MoveToLevel(Level level);
  frc2::CommandPtr CollectCoral();
  frc2::CommandPtr PlaceCoral();
  
  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ElevatorSubsystem& m_elevatorSubsystem;
  ArmSubsystem& m_armSubsystem;
};

