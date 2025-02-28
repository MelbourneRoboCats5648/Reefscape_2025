#pragma once
#include <frc2/command/SubsystemBase.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/Commands.h>
#include <Constants.h>
#include <rev/SparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/DigitalInput.h>

class ElevatorSubsystem : public frc2::SubsystemBase {
  public:
  ElevatorSubsystem();
  
  units::meter_t GetElevatorHeight();
  frc::TrapezoidProfile<units::meter>::State& GetSetpoint();
  frc::TrapezoidProfile<units::meter>::State& GetGoal();
  bool IsGoalReached();

  void ResetMotor();
  void OnLimitSwitchActivation();
  void ResetEncoder();
  //Elevator command factory method.
  frc2::CommandPtr MoveDownCommand();
  frc2::CommandPtr MoveUpCommand();
  frc2::CommandPtr MoveToHeightCommand(units::meter_t heightGoal);
  frc2::CommandPtr MoveUpBy(units::meter_t height);

  //Will be called periodically whenever the CommandScheduler runs.
  void Periodic() override;

  //Will be called periodically whenever the CommandScheduler runs during simulation.
  void SimulationPeriodic() override;
  
  // Will be called periodically.
  private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  
  //digital input
  frc::DigitalInput m_limitSwitchElevator{ElevatorConstants::k_limitSwitchElevatorPin};
  
  // Spark components
  //plan to add motors to hard switches
  rev::spark::SparkMax m_motorFirstStageLeft{CAN_Constants::kElevatorMotorLeftCAN_ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_motorFirstStageRight{CAN_Constants::kElevatorMotorRightCAN_ID, rev::spark::SparkMax::MotorType::kBrushless}; 
  rev::spark::SparkMax m_motorSecondStage{CAN_Constants::kElevatorMotorSecondStageCAN_ID, rev::spark::SparkMax::MotorType::kBrushless}; 
  rev::spark::SparkRelativeEncoder m_encoderLeft = m_motorFirstStageLeft.GetEncoder();
  rev::spark::SparkRelativeEncoder m_encoderRight = m_motorFirstStageRight.GetEncoder(); 
  rev::spark::SparkRelativeEncoder m_encoderSecondStage = m_motorSecondStage.GetEncoder();      

  //frc::SimpleMotorFeedforward<units::meters> m_feedforward{ 
  /* Note: These gains are fake, and will have to be tuned for your robot. feedforward will be used soon
  * 1_V, 1.5_V * 1_s / 1_m}; */

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  frc::TrapezoidProfile<units::meter> m_trapezoidalProfile{{ElevatorConstants::maximumVelocity, ElevatorConstants::maximumAcceleration}};
  frc::TrapezoidProfile<units::meter>::State m_elevatorGoal;
  frc::TrapezoidProfile<units::meter>::State m_elevatorSetpoint;
  rev::spark::SparkClosedLoopController m_closedLoopControllerLeft = m_motorFirstStageLeft.GetClosedLoopController();
  rev::spark::SparkClosedLoopController m_closedLoopControllerSecondStage = m_motorSecondStage.GetClosedLoopController();

  // Create a new ElevatorFeedforward with gains kS, kV, and kA
  // Distance is measured in meters
  frc::ElevatorFeedforward m_elevatorFeedforward{ElevatorConstants::kS, ElevatorConstants::kG, ElevatorConstants::kV, ElevatorConstants::kA};
  
  void UpdateSetpoint();
  frc2::CommandPtr MoveFirstStageToHeightCommand(units::meter_t goal);
  frc2::CommandPtr MoveSecondStageToHeightCommand(units::meter_t goal);
};




