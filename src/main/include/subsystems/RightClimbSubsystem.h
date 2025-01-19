// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once


#include <frc/Encoder.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/VictorSP.h>
#include <frc/Joystick.h>
#include <rev/SparkMax.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
// possibly add smart dashboard from example for hard switches
#include <rev/config/SparkMaxConfig.h>
#include <frc2/command/Commands.h>

#include <frc/controller/SimpleMotorFeedforward.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>


using namespace rev::spark;

//FRC Motor Ports
// Changed to 1 because 2024 motors have become redundant- plan to delete
const int motorClimbRightPort= 1;

//Spark Motor ID 
// Plan to change when motors are organised
const int motorClimbRightID = 1;

// Speeds
const double rightClimbUpSpeed = 1.0; //was 0.25
const double rightClimbDownSpeed = -1.0;

// Joystick buttons
const int rightUpButton = 6;
const int rightDownButton = 4;

// Soft Limits
// Plan to change from example base when limits are decided
const int  extendSoftLimit = 50;
const int  retractSoftLimit= -50;

namespace RightClimbConstants {
//PID Controller
const double kP = 0.0;
const double kI = 0.0;
const double kD = 0.0;


//PID Profile 
/*const units::turns_per_second_t maximumVelocity = 1.75_tps;
const units::turns_per_second_squared_t maximumAccelaration = 0.75_tr_per_s_sq;*/

const units::radians_per_second_t maximumVelocity = 1.75_rad_per_s;
const units::radians_per_second_squared_t maximumAcceleration = 0.75_rad_per_s_sq;
//kDt
const units::second_t kDt = 20_ms;

const units::turn_t kGoalThreshold = 0.1_tr;
}



class RightClimbSubsystem : public frc2::SubsystemBase {
 public:
  RightClimbSubsystem();
 
         
  /**
   * Command factory method.
   */
  frc2::CommandPtr RightClimbUpCommand();
  frc2::CommandPtr RightClimbDownCommand();
  
  /**
   * Returns a command to shoot the balls currently stored in the robot. Spins
   * the shooter flywheel up to the specified setpoint, and then runs the feeder
   * motor.
   *
   * @param setpointRotationsPerSecond The desired right climb velocity
   */
  [[nodiscard]]
  frc2::CommandPtr RightClimbCommand(units::turn_t goal);



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

//old frc 2024 season motors redundant (need to change to spark)
          frc::VictorSP m_motorClimbRight{motorClimbRightPort};

// Spark components
//plan to add motors and hard switches
          SparkMax m_motorController{motorClimbRightID, SparkMax::MotorType::kBrushless};
          SparkClosedLoopController m_closedLoopController = m_motorController.GetClosedLoopController();
          SparkRelativeEncoder m_encoder = m_motorController.GetEncoder();

          // Create a motion profile with the given maximum velocity and maximum
          // acceleration constraints for the next setpoint.
          frc::TrapezoidProfile<units::radians_per_second> m_profile{{RightClimbConstants::maximumVelocity, RightClimbConstants::maximumAcceleration}};
          
         
          

};