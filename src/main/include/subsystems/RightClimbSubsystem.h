// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/VictorSP.h>
#include <frc/Joystick.h>
#include <rev/SparkMax.h>
#include <frc/controller/PIDController.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
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


using namespace rev::spark;

//FRC Motor Ports

const int motorClimbRightPort= 0;

//SparkMax Motor ID
//plan to change with more Spark motors added
const int motorClimbRightID = 1;

// Speeds
const double rightClimbUpSpeed = 1.0; //was 0.25
const double rightClimbDownSpeed = -1.0;

// Joystick buttons
const int rightUpButton = 6;
const int rightDownButton = 4;

// Soft Limits
//plant to change from example base limits when limits are changed
const int  extendSoftLimit = 50;
const int  retractSoftLimit= -50;

namespace RightClimbConstants {
//PID Trapezoidal Controller
static constexpr units::second_t kDt = 20_ms;

//PID Controller
const double kP = 0.0;
const double kI = 0.0;
const double kD = 0.0;

//PID Profile] 
const units:: turns_per_second_t maximumVelocity= 1.75_tps;
const units::turns_per_second_squared_t maximumAccelaration = 0.75_tr_per_s_sq;

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
//plan to add motors to hard switches
          SparkMax m_motor{motorClimbRightID, SparkMax::MotorType::kBrushless};
          SparkRelativeEncoder m_encoder = m_motor.GetEncoder();
           

    
    /*frc::SimpleMotorFeedforward<units::meters> m_feedforward{
      // Note: These gains are fake, and will have to be tuned for your robot. feedforward will be used soon
      //1_V, 1.5_V * 1_s / 1_m}; */

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  frc::TrapezoidProfile<units::meters> m_profile{{1.75_mps, 0.75_mps_sq}};
  frc::TrapezoidProfile<units::meters>::State m_goal;
  frc::TrapezoidProfile<units::meters>::State m_setpoint;
  SparkClosedLoopController m_closedLoopController = m_motor.GetClosedLoopController();

  //Edited Trapezoid Profiler with turn_t doesn't build
  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  //frc::TrapezoidProfile<units::turn_t> m_profile{{RightClimbConstants::maximumVelocity, RightClimbConstants::maximumAccelaration}};
  
};
