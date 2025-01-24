#pragma once
#include <numbers>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <Constants.h>
#include <frc/controller/PIDController.h>
// possibly add smart dashboard from example for hard switches
#include <rev/config/SparkMaxConfig.h>
// PID Profile and Controller stuff
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/Commands.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

 //SparkMax Motor ID
//plan to change with more Spark motors added
const int motorElevatorID = 1;

// Soft Limits- will port to elevator and drive soon
//plant to change from example base limits when limits are changed
const units::turn_t extendSoftLimit = 50_tr;
const units::turn_t retractSoftLimit= -50_tr;



namespace ElevatorConstants {
//PID Trapezoidal Controller
static constexpr units::second_t kDt = 20_ms;
const units::turn_t kGoalThreshold = 3.0_tr; //part of RightClimbCommand

//PID Controller
const double kP = 1.0;
const double kI = 0.0;
const double kD = 0.0;

//PID Profile] 
const units::turns_per_second_t maximumVelocity= 1.5_tps;
const units::turns_per_second_squared_t maximumAccelaration = 1.0_tr_per_s_sq;

//Elevator Goals
const units::turn_t level1Goal = 2.0_tr;
const units::turn_t level2Goal = 3.0_tr;
const units::turn_t level3Goal = 4.0_tr;

}

class ElevatorSubsystem : public frc2::SubsystemBase {
  private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Initialize the motor
  rev::spark::SparkMax m_elevatorLiftMotor{CAN_Constants::kElevatorMotorCAN_ID, rev::spark::SparkMax::MotorType::kBrushless};

  public:
  ElevatorSubsystem();

  /**
   * Elevator command factory method.
   */
  frc2::CommandPtr MoveUpToL1Command(units::turn_t goal);
  frc2::CommandPtr MoveUpToL2Command(units::turn_t goal);
  frc2::CommandPtr MoveUpToL3Command(units::turn_t goal);
  frc2::CommandPtr MoveDownCommand();
  frc2::CommandPtr ElevatorCommand(units::turn_t goal);


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
  rev::spark::SparkMax m_motor{motorElevatorID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkRelativeEncoder m_encoder = m_motor.GetEncoder();
           

  /*frc::SimpleMotorFeedforward<units::meters> m_feedforward{
      // Note: These gains are fake, and will have to be tuned for your robot. feedforward will be used soon
      //1_V, 1.5_V * 1_s / 1_m}; */

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  //frc::TrapezoidProfile<units::meters> m_profile{{1.75_mps, 0.75_mps_sq}};
  frc::TrapezoidProfile<units::turn> m_trapezoidalProfile{{ElevatorConstants::maximumVelocity, ElevatorConstants::maximumAccelaration}};
  frc::TrapezoidProfile<units::turn>::State m_elevatorGoal;
  frc::TrapezoidProfile<units::turn>::State m_elevatorSetpoint;
  rev::spark::SparkClosedLoopController m_closedLoopController = m_elevatorLiftMotor.GetClosedLoopController();

};




