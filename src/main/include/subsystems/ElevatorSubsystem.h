#pragma once
#include <frc2/command/SubsystemBase.h>
//#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/Commands.h>
#include <Constants.h>
#include <rev/SparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/ElevatorFeedforward.h>

class ElevatorSubsystem : public frc2::SubsystemBase {
  public:
  ElevatorSubsystem();

  /**
   * Elevator command factory method.
   */
  frc2::CommandPtr MoveDownCommand();
  frc2::CommandPtr MoveUpCommand();
  frc2::CommandPtr MoveToHeightCommand(units::meter_t goal);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  private:
   // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

 // Spark components
 //plan to add motors to hard switches
  rev::spark::SparkMax m_motor{CAN_Constants::kElevatorMotorCAN_ID, rev::spark::SparkMax::MotorType::kBrushless}; // issue 64 - the elevator has 2 motors connected to each other
  rev::spark::SparkRelativeEncoder m_encoder = m_motor.GetEncoder();
           

  /*frc::SimpleMotorFeedforward<units::meters> m_feedforward{
// Note: These gains are fake, and will have to be tuned for your robot. feedforward will be used soon
//1_V, 1.5_V * 1_s / 1_m}; */

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  frc::TrapezoidProfile<units::meter> m_trapezoidalProfile{{ElevatorConstants::maximumVelocity, ElevatorConstants::maximumAcceleration}};
  frc::TrapezoidProfile<units::meter>::State m_elevatorGoal;
  frc::TrapezoidProfile<units::meter>::State m_elevatorSetpoint;
  rev::spark::SparkClosedLoopController m_closedLoopController = m_motor.GetClosedLoopController();
// Create a new ElevatorFeedforward with gains kS, kV, and kA
// Distance is measured in meters
  frc::ElevatorFeedforward m_elevatorFeedforward{ElevatorConstants::kS, ElevatorConstants::kG, ElevatorConstants::kV, ElevatorConstants::kA};
  
  void UpdateSetpoint();
  void ResetMotor();
  units::meter_t GetElevatorPosition();
};




