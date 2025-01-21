// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <numbers>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/Joystick.h>
#include <frc/motorcontrol/VictorSP.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <frc/controller/PIDController.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <rev/config/SparkMaxConfig.h>
#include <frc2/command/Commands.h>

#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

using namespace rev::spark;

// PWM Ports
const int motorClimbLeftPort = 1;

//Motor ID
const int motorClimbLeftID = 1;

// Speeds
const double leftClimbUpSpeed = 1.0; //was 0.25
const double leftClimbDownSpeed = -1.0;

// Joystick buttons
const int leftUpButton = 3;
const int leftDownButton = 5;

// Soft Limits
// Plan to change from example base when limits are decided
const int  extendSoftLimit = 50;
const int  retractSoftLimit= -50;

//PID Controller constants
namespace LeftClimbConstants 
{
const double kP = 1.0;
const double kI = 0.0;
const double kD = 0.0;

//PID Profile 
const units::turns_per_second_t maximumVelocity = 0.5_tps;
const units::turns_per_second_squared_t maximumAcceleration = 0.25_tr_per_s_sq;
//kDt
const units::second_t kDt = 20_ms;
const units::turn_t kGoalThreshold = 3.0_tr;
}

class LeftClimbSubsystem : public frc2::SubsystemBase {
 public:
  LeftClimbSubsystem();

  /**
   * LeftClimb command factory method.
   */
  frc2::CommandPtr LeftClimbUpCommand();
  frc2::CommandPtr LeftClimbDownCommand();
  frc2::CommandPtr LeftClimbL1Command(units::turn_t goal);

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
      frc::VictorSP m_motorClimbLeft{motorClimbLeftPort};
      SparkMax m_motorController{motorClimbLeftID, SparkMax::MotorType::kBrushless};
      SparkClosedLoopController m_closedLoopController = m_motorController.GetClosedLoopController();
      SparkRelativeEncoder m_encoder = m_motorController.GetEncoder();
      frc::TrapezoidProfile<units::turn> m_TrapezoidalProfile{{LeftClimbConstants::maximumVelocity, LeftClimbConstants::maximumAcceleration}};
      frc::TrapezoidProfile<units::turn>::State m_leftClimbGoal;
      frc::TrapezoidProfile<units::turn>::State m_leftClimbSetpoint;

};
