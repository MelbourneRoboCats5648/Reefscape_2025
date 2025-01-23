// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

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

inline constexpr int kElevatorLeftMotorCAN_ID = 1;

//kElevatorRightMotorCAN_ID is currently not configurated
inline constexpr int kElevatorRightMotorCAN_ID = 2;

}  // namespace OperatorConstants
