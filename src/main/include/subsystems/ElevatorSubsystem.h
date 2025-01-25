#pragma once
#include <Constants.h>
#include <frc/smartdashboard/SmartDashboard.h>

class ElevatorSubsystem : public frc2::SubsystemBase {
  public:
  ElevatorSubsystem();

  /**
   * Elevator command factory method.
   */
  frc2::CommandPtr MoveDownCommand();
  frc2::CommandPtr MoveUpCommand();
  frc2::CommandPtr MoveToLevelCommand(units::turn_t goal);
  
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
  rev::spark::SparkMax m_motor{CAN_Constants::kElevatorMotorCAN_ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkRelativeEncoder m_encoder = m_motor.GetEncoder();
           

  /*frc::SimpleMotorFeedforward<units::meters> m_feedforward{
      // Note: These gains are fake, and will have to be tuned for your robot. feedforward will be used soon
      //1_V, 1.5_V * 1_s / 1_m}; */

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  //frc::TrapezoidProfile<units::meters> m_profile{{1.75_mps, 0.75_mps_sq}};
  frc::TrapezoidProfile<units::turn> m_trapezoidalProfile{{ElevatorConstants::maximumVelocity, ElevatorConstants::maximumAcceleration}};
  frc::TrapezoidProfile<units::turn>::State m_elevatorGoal;
  frc::TrapezoidProfile<units::turn>::State m_elevatorSetpoint;
  rev::spark::SparkClosedLoopController m_closedLoopController = m_motor.GetClosedLoopController();

};




