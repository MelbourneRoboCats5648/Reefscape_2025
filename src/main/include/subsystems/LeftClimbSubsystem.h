// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "Constants.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>

// PWM Ports
//const int motorClimbLeftPort = 1;

class LeftClimbSubsystem : public frc2::SubsystemBase {
 public:
  LeftClimbSubsystem();

  /**
   * LeftClimb command factory method.
   */
  frc2::CommandPtr LeftClimbUpCommand();
  frc2::CommandPtr LeftClimbDownCommand();
  
  frc2::CommandPtr LeftClimbDefaultCommand(units::turn_t goal);
  frc2::CommandPtr MoveToClimbLevel(ClimbLevel climbLevel);

  void StopMotor();
  void ResetEncoder();

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
      //frc::VictorSP m_motorClimbLeft{motorClimbLeftPort};
      rev::spark::SparkMax m_motorController{CAN_Constants::kClimbCAN_ID, rev::spark::SparkMax::MotorType::kBrushless};
      rev::spark::SparkClosedLoopController m_closedLoopController = m_motorController.GetClosedLoopController();
      rev::spark::SparkRelativeEncoder m_encoder = m_motorController.GetEncoder();
      frc::TrapezoidProfile<units::turn> m_TrapezoidalProfile{{LeftClimbConstants::maximumVelocity, LeftClimbConstants::maximumAcceleration}};
      frc::TrapezoidProfile<units::turn>::State m_leftClimbGoal;
      frc::TrapezoidProfile<units::turn>::State m_leftClimbSetpoint;
      void RobotReset();
};
