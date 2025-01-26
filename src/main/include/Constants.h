// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

inline constexpr int kDriveJoystickPort = 0;
inline constexpr int kDriverControllerPort = 1;

}  // namespace OperatorConstants

namespace CAN_Constants {

inline constexpr int kElevatorMotorCAN_ID = 1;

}  // namespace OperatorConstants

namespace GoalConstants {
  inline constexpr units::turn_t m_climbGoalL1 = 1.0_tr; 
  inline constexpr units::turn_t m_climbGoalRetract = 0.0_tr; 
}

namespace LeftClimbConstants {
  //Motor ID
  const int motorClimbLeftID = 1;
  // Speeds
  const double leftClimbUpSpeed = 1.0; //was 0.25
  const double leftClimbDownSpeed = -1.0;
  // Joystick buttons
  const int leftUpButton = 3;
  const int leftDownButton = 5;
  // Soft Limits - Plan to change from example base when limits are decided
  const int  extendSoftLimit = 50;
  const int  retractSoftLimit= -50;
  //PID Controller constants
  const double kP = 1.0;
  const double kI = 0.0;
  const double kD = 0.0;
  //PID Profile 
  const units::turns_per_second_t maximumVelocity = 1.5_tps;
  const units::turns_per_second_squared_t maximumAcceleration = 1.0_tr_per_s_sq;
  //kDt
  const units::second_t kDt = 20_ms;
  const units::turn_t kGoalThreshold = 3.0_tr;
}