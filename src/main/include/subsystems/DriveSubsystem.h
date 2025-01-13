#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <units/angle.h>
#include <frc/ADIS16470_IMU.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/geometry/Translation2d.h>
#include <frc/controller/ProfiledPIDController.h>

#include <frc/geometry/Rotation2d.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>

#include "utilities/DriveModule.h"

#include "Constants.h"

using namespace DriveConstants;


class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

    void Periodic() override;
    void SimulationPeriodic() override;

    void Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot, bool fieldRelative,
                           units::second_t period = DriveConstants::kDrivePeriod);

    void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);
    void StopAllModules();
    frc2::CommandPtr StopCommand();
    double GetPositionDistance();
    void SetPositionToZeroDistance();
    
      /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  units::degree_t GetHeading() const;

  /**
   * Zeroes the heading of the robot.
   */
  void ResetGyro();


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