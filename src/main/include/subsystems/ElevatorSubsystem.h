#pragma once
#include <frc2/command/SubsystemBase.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/Commands.h>
#include <Constants.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <subsystems/ElevatorStageSubsystem.h>

class ElevatorSubsystem : public frc2::SubsystemBase {
  public:
  ElevatorSubsystem();

  units::meter_t GetElevatorHeight();

  frc2::CommandPtr MoveUpCommand();
  frc2::CommandPtr MoveDownCommand();

  frc2::CommandPtr MoveToHeightCommand(units::meter_t heightGoal);
  frc2::CommandPtr MoveUpBy(units::meter_t height);

  frc2::CommandPtr CancelFirstStageCommand();
  frc2::CommandPtr CancelSecondStageCommand();

  //Will be called periodically whenever the CommandScheduler runs.
  void Periodic() override;

  //Will be called periodically whenever the CommandScheduler runs during simulation.
  void SimulationPeriodic() override;
  
  ElevatorStageSubsystem m_firstStage{
    "ElevatorStage1",
    ElevatorConstants::retractSoftLimitFirstStage, ElevatorConstants::kMaxFirstStageHeight,
    ElevatorConstants::kInitFirstStageHeight, ElevatorConstants::kResetFirstStageHeight,
    ElevatorConstants::distancePerTurnFirstStage,
    ElevatorConstants::kFirstStagePID, ElevatorConstants::kFirstStageFeedforward,
    ElevatorConstants::trapezoidProfileFirstStage,
    ElevatorConstants::kFirstStageLimitSwitchPin,
    CAN_Constants::kElevatorMotorLeftCAN_ID, CAN_Constants::kElevatorMotorRightCAN_ID
  };
  ElevatorStageSubsystem m_secondStage{
    "ElevatorStage2",
    ElevatorConstants::retractSoftLimitSecondStage, ElevatorConstants::kMaxSecondStageHeight,
    ElevatorConstants::kInitSecondStageHeight, ElevatorConstants::kResetSecondStageHeight,
    ElevatorConstants::distancePerTurnSecondStage,
    ElevatorConstants::kSecondStagePID, ElevatorConstants::kSecondStageFeedforward,
    ElevatorConstants::trapezoidProfileSecondStage,
    ElevatorConstants::kSecondStageLimitSwitchPin,
    CAN_Constants::kElevatorMotorSecondStageCAN_ID
  };
};




