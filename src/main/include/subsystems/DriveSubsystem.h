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

#include <frc/estimator/PoseEstimator.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <wpi/array.h>

using namespace DriveConstants;


class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

    void Periodic() override;
    void SimulationPeriodic() override;

  //Gyro Styff
    void ResetGyro();

    units::degree_t GetHeading() const;

  //Drive and Kinematics 
    void Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot, bool fieldRelative,
                           units::second_t period = kDrivePeriod);

    void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  //Stops
    void StopAllModules();
    frc2::CommandPtr StopCommand();

  // Odometry
      /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
    frc::Pose2d GetPose();

      /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

  void SetPositionToZeroDistance();



 private:
    //Gyro
    frc::ADIS16470_IMU m_gyro;
    //Translation2D
    frc::Translation2d backLeftLocation{-0.26_m, +0.26_m};
    frc::Translation2d backRightLocation{-0.26_m, -0.26_m};
    frc::Translation2d frontLeftLocation{+0.26_m, +0.26_m};
    frc::Translation2d frontRightLocation{+0.26_m, -0.26_m};

    //MagOffset Doubles
      const double FRONT_LEFT_MAG_OFFSET = (0.303955);
      const double FRONT_RIGHT_MAG_OFFSET = (0.346436);
      const double BACK_LEFT_MAG_OFFSET = (0.064697);
      const double BACK_RIGHT_MAG_OFFSET = (0.015381); 
      // this mag offset has been set by the phoenix tuner set the new offsets

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
    // Odometry class for tracking robot pose
    // 4 defines the number of modules
    frc::SwerveDriveOdometry<4> m_odometry;


    frc::SwerveDrivePoseEstimator<4> m_poseEstimator{kinematics, frc::Rotation2d{GetHeading()},
                                {m_frontLeftModule.GetPosition(), m_frontRightModule.GetPosition(),
                                  m_backLeftModule.GetPosition(), m_backRightModule.GetPosition()}, 
                                  GetPose(), 
                                  // ( double 0.5, double 0.5, units::radian_t(1)), 
                                  // ( double 0.5, double 0.5, units::radian_t(1))
                                  };

};