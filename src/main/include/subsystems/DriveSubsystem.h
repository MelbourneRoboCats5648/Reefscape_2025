#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <units/angle.h>
#include <frc/ADIS16470_IMU.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/geometry/Translation2d.h>
#include <frc/controller/ProfiledPIDController.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>

#include "utilities/DriveModule.h"

const int FRONT_LEFT_SPEED_MOTOR_ID = 4;
const int FRONT_RIGHT_SPEED_MOTOR_ID = 2;
const int BACK_LEFT_SPEED_MOTOR_ID = 6;
const int BACK_RIGHT_SPEED_MOTOR_ID = 7;

const int FRONT_LEFT_DIRECTION_MOTOR_ID = 3;
const int FRONT_RIGHT_DIRECTION_MOTOR_ID = 5;
const int BACK_LEFT_DIRECTION_MOTOR_ID = 1;
const int BACK_RIGHT_DIRECTION_MOTOR_ID = 8;

const int FRONT_LEFT_DIRECTION_ENCODER_ID = 9;
const int FRONT_RIGHT_DIRECTION_ENCODER_ID = 10;
const int BACK_LEFT_DIRECTION_ENCODER_ID = 12;
const int BACK_RIGHT_DIRECTION_ENCODER_ID = 11;

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  /**
   * Example command factory method.
   */
  frc2::CommandPtr ExampleMethodCommand();

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  bool ExampleCondition();

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
    //Gyro
    frc::ADIS16470_IMU m_gyro;
    //Translation2D
    frc::Translation2d backLeftLocation{-0.26_m, +0.26_m};
    frc::Translation2d backRightLocation{-0.26_m, -0.26_m};
    frc::Translation2d frontLeftLocation{+0.26_m, +0.26_m};
    frc::Translation2d frontRightLocation{+0.26_m, -0.26_m};

    //MagOffset Doubles
    const double FRONT_LEFT_MAG_OFFSET = 0.2939453125;
    const double FRONT_RIGHT_MAG_OFFSET = 0.346435546875;
    const double BACK_LEFT_MAG_OFFSET = 0.06201171875;
    const double BACK_RIGHT_MAG_OFFSET = 0.012451171875;

    DriveModule m_frontLeftModule{FRONT_LEFT_SPEED_MOTOR_ID, FRONT_LEFT_DIRECTION_MOTOR_ID, 
                                    FRONT_LEFT_DIRECTION_ENCODER_ID, FRONT_LEFT_MAG_OFFSET, "Front Left"};
    DriveModule m_frontRightModule{FRONT_RIGHT_SPEED_MOTOR_ID, FRONT_RIGHT_DIRECTION_MOTOR_ID, 
                                    FRONT_RIGHT_DIRECTION_ENCODER_ID, FRONT_RIGHT_MAG_OFFSET, "Front Right"};
    DriveModule m_backLeftModule{BACK_LEFT_SPEED_MOTOR_ID, BACK_LEFT_DIRECTION_MOTOR_ID, 
                                    BACK_LEFT_DIRECTION_ENCODER_ID, BACK_LEFT_MAG_OFFSET, "Back Left"};
    DriveModule m_backRightModule{BACK_RIGHT_SPEED_MOTOR_ID, BACK_RIGHT_DIRECTION_MOTOR_ID, 
                                    BACK_RIGHT_DIRECTION_ENCODER_ID, BACK_RIGHT_MAG_OFFSET, "Back Right"};
    
    frc::SwerveDriveKinematics<4> kinematics{frontLeftLocation, 
                                            frontRightLocation, 
                                            backLeftLocation,
                                            backRightLocation};
                                            
};