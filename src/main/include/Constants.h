#pragma once
//SparkMax
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
//Feedforward + Motion profiling
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>
// PID Profile and Controller stuff
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <frc/TimedRobot.h>
#include <frc/geometry/Translation2d.h>


enum BuildSeason {Crescendo, Reefscape};

enum Level {L0, L1, L2, L3, L4};

namespace General {
// Choose the bindings for which robot to build
const BuildSeason KBuildSeason = BuildSeason::Crescendo;
}

namespace OperatorConstants {  
inline constexpr int kDriverControllerPort = 0;
inline constexpr int kDriverJoystickPort = 1;
inline constexpr units::meters_per_second_squared_t kSlewRateTranslation = 6_mps_sq; //increase to reduce lag
inline constexpr units::radians_per_second_squared_t kSlewRateRotation = 6_rad_per_s_sq;
inline constexpr double kDeadband = 0.1;
}  // namespace OperatorConstants

namespace CAN_Constants {
//Subsystem CAN IDs
inline constexpr int kElevatorMotorLeftCAN_ID = 13;
inline constexpr int kElevatorMotorRightCAN_ID = 15;
// not configurated yet
inline constexpr int kElevatorArmMotorCAN_ID = 14;

//Drive CAN IDs
inline constexpr int kFrontLeftSpeedMotorID = 4;
inline constexpr int kFrontRightSpeedMotorID = 2;
inline constexpr int kBackLeftSpeedMotorID = 6;
inline constexpr int kBackRightSpeedMotorID = 7;
inline constexpr int kFrontLeftDirectionMotorID = 3;
inline constexpr int kFrontRightDirectionMotorID = 5;
inline constexpr int kBackLeftDirectionMotorID = 1;
inline constexpr int kBackRightDirectionMotorID = 8;
inline constexpr int kFrontLeftDirectionEncoderID = 9;
inline constexpr int kFrontRightDirectionEncoderID = 10;
inline constexpr int kBackLeftDirectionEncoderID = 12;
inline constexpr int kBackRightDirectionEncoderID = 11;

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

namespace DriveConstants {
//Max Speed and Acceleration Constanst
inline constexpr auto kMaxSpeed = 3_mps;
inline constexpr auto kMaxAcceleration = 3_mps_sq;
inline constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
inline constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

inline constexpr units::second_t kDrivePeriod = frc::TimedRobot::kDefaultPeriod;

// Mechanical Constants
inline constexpr double kTurningGearRatio = 150.0 / 7.0;
inline constexpr double kDriveGearRatio = 6.75;  // L2 - Fast kit check the gear ratio
inline constexpr units::meter_t kWheelRadius = 0.0508_m;
inline constexpr units::meter_t kWheelCircumference = 2 * std::numbers::pi * kWheelRadius;

//Module constants defining
inline constexpr auto kModuleMaxAngularVelocity =
    std::numbers::pi * 4_rad_per_s;  // radians per second

inline constexpr auto kModuleMaxAngularAcceleration =
    std::numbers::pi * 8_rad_per_s / 1_s;  // radians per second^2

//Speed Motor Config PID
inline constexpr double kSpeedMotorKP = 0.3;
inline constexpr double kSpeedMotorKI = 0.0;
inline constexpr double kSpeedMotorKD = 0.0;
inline constexpr double kSpeedMotorkV = 0.1;

//Turn PID
inline constexpr double kTurnKP = 6.0;
inline constexpr double kTurnKI = 0.0;
inline constexpr double kTurnKD = 0.0;

//MagOffset Doubles
inline constexpr units::angle::turn_t kFrontLeftMagOffset = 0.302246_tr;
inline constexpr units::angle::turn_t kFrontRightMagOffset = 0.351074_tr;
inline constexpr units::angle::turn_t kBackLeftMagOffset = 0.062012_tr;
inline constexpr units::angle::turn_t kBackRightMagOffset = 0.012695_tr; 

//Module Locations Translation 2D
inline constexpr frc::Translation2d kFrontLeftLocation{+0.26_m, +0.26_m};
inline constexpr frc::Translation2d kFrontRightLocation{+0.26_m, -0.26_m};
inline constexpr frc::Translation2d kBackLeftLocation{-0.26_m, +0.26_m};
inline constexpr frc::Translation2d kBackRightLocation{-0.26_m, -0.26_m};
}

namespace ElevatorConstants {
//PID Trapezoidal Controller
static constexpr units::second_t kDt = 20_ms;
const units::meter_t kGoalThreshold = 3.0_m;

//Smart Current Limit
const int kCurrentLimit = 50;

//PID Controller
const double kP = 0.1;
const double kI = 0.0;
const double kD = 0.0;
const double maxOutput = 1.0;

//PID Profile
const units::meters_per_second_t maximumVelocity= 0.5_mps;
const units::meters_per_second_squared_t maximumAcceleration = 1.0_mps_sq;

//Elevator Goals
const units::meter_t eLevel0Goal = 0.0_m;
const units::meter_t eLevel1Goal = eLevel0Goal;
const units::meter_t eLevel2Goal = 0.5_m;
const units::meter_t eLevel3Goal = 1.0_m;
const units::meter_t eLevel4Goal = 1.5_m;

//Encoder Position
const units::turn_t resetEncoder = 0.0_tr;

// Elevator limits
const units::meter_t extendSoftLimit = 7.0_m;

//Elevator feedforward
const units::volt_t kS = 1.0_V;
const units::volt_t kG = 1.0_V;
const auto kV = 1.0_V / 1_mps;
const auto kA = 1.0_V / 1_mps_sq;

//Elevator Height Conversion
const units::meter_t distancePerTurn = 0.1_m;

const double gearRatio = 1.0 / 9.0; // issue 64 - update this param

const double k_elevatorPositionTolerance = 0.1; // issue 70 - update this tolerance
const double k_elevatorVelocityTolerance = 0.1;
const units::meter_t kElevatorMinHeightCollect = 1_m; //issue 70 - update this position
const units::meter_t kElevatorPlaceCoral = 0.1_m; // issue 70 - update this amount

}

namespace ArmConstants {
//PID Profile
const units::turns_per_second_t maximumVelocity= 0.5_tps;
const units::turns_per_second_squared_t maximumAcceleration = 1.0_tr_per_s_sq;

//PID Trapezoidal Controller
static constexpr units::second_t kDt = 20_ms;
const units::turn_t kGoalThreshold = 3.0_tr;

// add arm soft limits

//Arm Goals - this is the output of the gearbox (not the motor)
const units::turn_t aLevel0Goal = -0.25_tr;
const units::turn_t aLevel1Goal = 0.0_tr;
const units::turn_t aLevel2Goal = 0.125_tr;
const units::turn_t aLevel3Goal = 0.125_tr;
const units::turn_t aLevel4Goal = 0.0_tr;

const double gearRatio = 1.0 / 9.0; // issue 64 - update this param

const double k_armPositionTolerance = 0.1; // issue 70 - update this tolerance
const double k_armVelocityTolerance = 0.1;
const units::turn_t kArmPlaceCoral = -15_tr; // issue 70 - update this amount

}

