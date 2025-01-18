#pragma once

#include <numbers>
#include <units/time.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

#include <frc/TimedRobot.h>


namespace OperatorConstants {
    inline constexpr int kDriverControllerPort = 0;
    inline constexpr int kDriveJoystickPort = 1;
}

namespace DriveConstants {

    inline constexpr const int FRONT_LEFT_SPEED_MOTOR_ID = 4;
    inline constexpr const int FRONT_RIGHT_SPEED_MOTOR_ID = 2;
    inline constexpr const int BACK_LEFT_SPEED_MOTOR_ID = 6;
    inline constexpr const int BACK_RIGHT_SPEED_MOTOR_ID = 7;

    inline constexpr const int FRONT_LEFT_DIRECTION_MOTOR_ID = 3;
    inline constexpr const int FRONT_RIGHT_DIRECTION_MOTOR_ID = 5;
    inline constexpr const int BACK_LEFT_DIRECTION_MOTOR_ID = 1;
    inline constexpr const int BACK_RIGHT_DIRECTION_MOTOR_ID = 8;

    inline constexpr const int FRONT_LEFT_DIRECTION_ENCODER_ID = 9;
    inline constexpr const int FRONT_RIGHT_DIRECTION_ENCODER_ID = 10;
    inline constexpr const int BACK_LEFT_DIRECTION_ENCODER_ID = 12;
    inline constexpr const int BACK_RIGHT_DIRECTION_ENCODER_ID = 11;

    inline constexpr auto kMaxSpeed = 3_mps;
    inline constexpr auto kMaxAcceleration = 3_mps_sq;
    inline constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
    inline constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

    inline constexpr units::second_t kDrivePeriod = frc::TimedRobot::kDefaultPeriod;

} 