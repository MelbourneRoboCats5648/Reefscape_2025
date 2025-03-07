#pragma once
//SparkMax
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
//Feedforward + Motion profiling
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/ArmFeedforward.h>
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
enum TestLevel {NONE, ARM, ELEVATOR, DRIVE};
enum ClimbLevel {C1, C2};

struct PIDConstants {
  double kP, kI, kD;
};

struct ArmFeedforwardConstants {
  units::volt_t kS, kG;
  units::unit_t<frc::ArmFeedforward::kv_unit> kV;
  units::unit_t<frc::ArmFeedforward::ka_unit> kA;
};

struct ElevatorFeedforwardConstants {
  units::volt_t kS, kG;
  units::unit_t<frc::ElevatorFeedforward::kv_unit> kV;
  units::unit_t<frc::ElevatorFeedforward::ka_unit> kA;
};

namespace General {
  // Choose the bindings for which robot to build
  const BuildSeason KBuildSeason = BuildSeason::Reefscape;
  const TestLevel KTestLevel = TestLevel::DRIVE;
}

// namespace OperatorConstants
namespace OperatorConstants {  
  inline constexpr int kDriverControllerPort = 0;
  inline constexpr int kDriverJoystickPort = 1;
  inline constexpr units::meters_per_second_squared_t kSlewRateTranslation = 6_mps_sq; //increase to reduce lag
  inline constexpr units::radians_per_second_squared_t kSlewRateRotation = 6_rad_per_s_sq;
  inline constexpr double kDeadband = 0.1;
}  

// namespace CAN Constants
namespace CAN_Constants {
  //Subsystem CAN IDs
  inline constexpr int kClimbCAN_ID = 20;
  inline constexpr int kElevatorMotorLeftCAN_ID = 21;
  inline constexpr int kElevatorMotorRightCAN_ID = 22;
  inline constexpr int kElevatorMotorSecondStageCAN_ID = 23;

  // not configurated yet
  inline constexpr int kElevatorArmMotorCAN_ID = 24;

  //Drive CAN IDs
  inline constexpr int kFrontLeftSpeedMotorID = 3;
  inline constexpr int kFrontRightSpeedMotorID = 1;
  inline constexpr int kBackLeftSpeedMotorID = 5;
  inline constexpr int kBackRightSpeedMotorID = 7;

  inline constexpr int kFrontLeftDirectionMotorID = 4;
  inline constexpr int kFrontRightDirectionMotorID = 2;
  inline constexpr int kBackLeftDirectionMotorID = 6;
  inline constexpr int kBackRightDirectionMotorID = 8;
  
  inline constexpr int kFrontLeftDirectionEncoderID = 11;
  inline constexpr int kFrontRightDirectionEncoderID = 10;
  inline constexpr int kBackLeftDirectionEncoderID = 12;
  inline constexpr int kBackRightDirectionEncoderID = 13;

  // Gyro CAN IDs
  const std::string kCanId = "rio";
  inline constexpr int kGyroDeviceID = 14; 
}  

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

  // Soft Limits
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
}

namespace DriveConstants {

  const units::angle::degree_t initialGyroAngle = 0_deg;

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
  inline constexpr units::angle::turn_t kFrontLeftMagOffset = -0.308349609375_tr;
  inline constexpr units::angle::turn_t kFrontRightMagOffset = -0.25732421875_tr;
  inline constexpr units::angle::turn_t kBackLeftMagOffset = 0.01611328125_tr;
  inline constexpr units::angle::turn_t kBackRightMagOffset = 0.0732421875_tr; 

  //Module Locations Translation 2D
  inline constexpr frc::Translation2d kFrontLeftLocation{+0.26_m, +0.26_m};
  inline constexpr frc::Translation2d kFrontRightLocation{+0.26_m, -0.26_m};
  inline constexpr frc::Translation2d kBackLeftLocation{-0.26_m, +0.26_m};
  inline constexpr frc::Translation2d kBackRightLocation{-0.26_m, -0.26_m};
}


namespace ElevatorConstants {
  //PID Trapezoidal Controller
  static constexpr units::second_t kDt = 20_ms;
  
  //Smart Current Limit
  const int kCurrentLimit = 50;

  //First Stage Controller
  const static PIDConstants kFirstStagePID = {
    /* kP */ 0.2,
    /* kI */ 0.0,
    /* kD */ 0.0
  };
  const static ElevatorFeedforwardConstants kFirstStageFeedforward = {
    /* kS */ 0.11_V,
    /* kG */ 0.1_V,
    /* kV */ 19.5_V / 1_mps,
    /* kA */ 0.0_V / 1_mps_sq
  };
  
  //Second Stage Controller
  const static PIDConstants kSecondStagePID = {
    /* kP */ 0.2,
    /* kI */ 0.0,
    /* kD */ 0.0
  };
  const static ElevatorFeedforwardConstants kSecondStageFeedforward = {
    /* kS */ 0.225_V,
    /* kG */ 0.0_V,
    /* kV */ 28.0_V / 1_mps,
    /* kA */ 0.0_V / 1_mps_sq
  };

  const double maxOutput = 1.0;

  //PID Profile
  static frc::TrapezoidProfile<units::meter> trapezoidProfileFirstStage{{0.5_mps, 2.0_mps_sq}};
  static frc::TrapezoidProfile<units::meter> trapezoidProfileSecondStage{{0.3_mps, 1.0_mps_sq}};

  //Elevator Goals
  const units::meter_t eLevel0Goal = 0.0_m;
  const units::meter_t eLevel1Goal = eLevel0Goal;
  const units::meter_t eLevel2Goal = 0.3_m;
  const units::meter_t eLevel3Goal = 0.6_m;
  const units::meter_t eLevel4Goal = 0.8_m;

  //Encoder Position
  const units::meter_t resetEncoder = 0.0_m;

  // Elevator limits
  const units::meter_t extendSoftLimitFirstStage = 0.68_m;
  const units::meter_t extendSoftLimitSecondStage = 0.62_m;

  const units::meter_t retractSoftLimitFirstStage = 0.0_m;
  const units::meter_t retractSoftLimitSecondStage = 0.38873863697052_m;


  // Maximum Elevator Heights
  const units::meter_t kMaxFirstStageHeight = extendSoftLimitFirstStage - 0.01_m;
  const units::meter_t kMaxSecondStageHeight = extendSoftLimitSecondStage - 0.01_m;

  // Initial Elevator heights
  const units::meter_t kInitFirstStageHeight = 0.0_m;
  const units::meter_t kInitSecondStageHeight = 0.37900087237358093_m;

  // Limit Switch Locations
  const units::meter_t kResetFirstStageHeight = 0.0_m;
  const units::meter_t kResetSecondStageHeight = 0.0_m;
  //Elevator Height Conversion:
  /* DIAMETERS OF THE MOTOR SPROCKETS:
  55 mm - 1st stage
  38 mm - 2nd stage
  CIRCUMFERENCE OF THE MOTOR SPROCKETS:
  0.1727876 m - 1st stage
  0.1193805 m - 2nd stage
  DISTANCE PER TURN = CIRCUMFERENCE */
  const units::meter_t distancePerTurnFirstStage = 0.1727876_m;
  const units::meter_t distancePerTurnSecondStage = 0.1193805_m;

  //Gear Ratio
  constexpr double gearBoxGearRatio = 1.0 / 27.0;

    constexpr double gearRatioFirstStage = gearBoxGearRatio * distancePerTurnFirstStage.value();
    constexpr double gearRatioSecondStage = gearBoxGearRatio * distancePerTurnSecondStage.value();


  const units::meter_t kElevatorPositionTolerance = 0.03_m;
  const units::meters_per_second_t kElevatorVelocityTolerance = 0.1_mps;

  const units::meter_t kElevatorMinHeightCollect = 1_m; //issue 70 - update this position
  const units::meter_t kElevatorPlaceCoral = 0.1_m; // issue 70 - update this amount
  
  //Elevator DIO port
  inline constexpr int kFirstStageLimitSwitchPin = 1;
  inline constexpr int kSecondStageLimitSwitchPin = 2; // TODO
}

namespace ArmConstants {
  //PID Profile
  const units::turns_per_second_t maximumVelocity= 0.8_tps;
  const units::turns_per_second_squared_t maximumAcceleration = 4.0_tr_per_s_sq;

  //PID Trapezoidal Controller
  static constexpr units::second_t kDt = 20_ms;

  //First Stage PID Controller 
  const double kP = 0.5;
  const double kI = 0.0;
  const double kD = 0.0;
  const double maxOutput = 1.0;

  //Arm feedforward
  const units::volt_t kS = 0.12_V;
  const units::volt_t kG = 0.25_V;
  const auto kV = 4.7_V / 1_tps;
  const auto kA = 0.0_V / 1_tr_per_s_sq;

  // Arm limits
  const units::turn_t extendSoftLimit = 0.10_tr;
  const units::turn_t retractSoftLimit = -0.23_tr;

  //Arm Goals - this is the output of the gearbox (not the motor)
  const units::turn_t aLevel0Goal = retractSoftLimit;
  const units::turn_t aLevel1Goal = 0.15_tr;
  const units::turn_t aLevel2Goal = 0.1_tr;
  const units::turn_t aLevel3Goal = 0.0_tr;
  const units::turn_t aLevel4Goal = -0.1_tr;

  constexpr double gearBoxGearRatio = 1.0 / 27.0;
  // this is the ratio between the motor sprocket teeth and the teeth on sprocket connected to the arm
  constexpr double motorSprocketRatio = 12.0 / 18.0;
  constexpr double gearRatio = gearBoxGearRatio * motorSprocketRatio;

  const double kArmPositionToleranceTurns = 0.01; // issue 70 - update this tolerance
  const double kArmVelocityTolerancePerSecond = 0.1;
  const units::turn_t kArmPlaceCoral = -15_tr; // issue 70 - update this amount

  //Encoder Position
  const units::turn_t resetEncoder = 0.15_tr;

  //Arm DIO port
  inline constexpr int k_limitSwitchArmPin = 3;
}

