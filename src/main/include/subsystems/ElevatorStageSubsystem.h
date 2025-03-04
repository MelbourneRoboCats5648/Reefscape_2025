#pragma once
#include <frc2/command/SubsystemBase.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/Commands.h>
#include <Constants.h>
#include <rev/SparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/DigitalInput.h>

class ElevatorStageSubsystem : public frc2::SubsystemBase {
  public:
  ElevatorStageSubsystem(
    std::string name,
    units::meter_t minLimit, units::meter_t maxLimit,
    units::meter_t initHeight, units::meter_t resetHeight,
    units::meter_t distancePerTurn,
    PIDConstants pidConst, ElevatorFeedforwardConstants ffConst,
    frc::TrapezoidProfile<units::meter>::Constraints pidProfile,
    int canID, int limitSwitchPin, int followerID = -1
  );
  
  units::meter_t GetHeight();
  units::meters_per_second_t GetVelocity();
  bool IsGoalReached();

  /* control using setpoint */
  void SetpointControl();
  void UpdateSetpoint();

  void ResetMotor();
  void ResetEncoder();
  void OnLimitSwitchActivation();

  /* elevator commands */
  frc2::CommandPtr SetpointControlCommand();
  frc2::CommandPtr MoveUpCommand();
  frc2::CommandPtr MoveDownCommand();
  frc2::CommandPtr MoveToHeightCommand(units::meter_t heightGoal);
  frc2::CommandPtr MoveUpBy(units::meter_t height);

  void Periodic() override;
  void SimulationPeriodic() override;

  private:
  const std::string m_name;

  //digital input
  frc::DigitalInput m_limitSwitch;
  
  rev::spark::SparkMax m_motor;
  rev::spark::SparkRelativeEncoder m_encoder;
  rev::spark::SparkClosedLoopController m_closedLoopController;

  frc::TrapezoidProfile<units::meter> m_trapezoidalProfile;
  frc::TrapezoidProfile<units::meter>::State m_goal;
  frc::TrapezoidProfile<units::meter>::State m_setpoint;

  frc::ElevatorFeedforward m_feedforward;

  const units::meter_t m_minLimit, m_maxLimit;
  const units::meter_t m_resetHeight;
  
  const double m_gearRatio;
};




