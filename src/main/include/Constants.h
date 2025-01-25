// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
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
inline constexpr int kDriverControllerPort2 = 2;

}  // namespace OperatorConstants

namespace CAN_Constants {

inline constexpr int kElevatorMotorCAN_ID = 1;

}  // namespace OperatorConstants

namespace ElevatorConstants {
//PID Trapezoidal Controller
static constexpr units::second_t kDt = 20_ms;
const units::turn_t kGoalThreshold = 3.0_tr; //part of RightClimbCommand

//PID Controller
const double kP = 1.0;
const double kI = 0.0;
const double kD = 0.0;

//PID Profile
const units::turns_per_second_t maximumVelocity= 0.5_tps;
const units::turns_per_second_squared_t maximumAcceleration = 1.0_tr_per_s_sq;

//Elevator Goals
const units::turn_t level1Goal = 2.0_tr;
const units::turn_t level2Goal = 3.0_tr;
const units::turn_t level3Goal = 4.0_tr;
const units::turn_t level4Goal = 0.0_tr;

//Encoder Position
const units::turn_t resetEncoder = 0.0_tr;

// Elevator limits
// Soft Limits- will port to elevator and drive soon
//plant to change from example base limits when limits are changed
const units::turn_t extendSoftLimit = 6.0_tr;
const units::turn_t retractSoftLimit = -1.0_tr;

}