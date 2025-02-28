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

#include <frc/filter/SlewRateLimiter.h>

#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

//Odometry 
#include <frc/estimator/PoseEstimator.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>

#include <wpi/array.h>

//Limelight
#include "LimelightHelpers.h"

//Namespaces
using namespace DriveConstants;
using namespace OperatorConstants;
using namespace CAN_Constants;


class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

    void Periodic() override;
    void SimulationPeriodic() override;

    //Gyro
      void ResetGyro();
      units::degree_t GetHeading() const;
    //Drive + Kinematics
      void Drive(units::meters_per_second_t xSpeed,
                            units::meters_per_second_t ySpeed,
                            units::radians_per_second_t rot, bool fieldRelative,
                            units::second_t period = DriveConstants::kDrivePeriod);

      void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

      frc::ChassisSpeeds GetRobotRelativeSpeeds();

    //Stops
      void StopAllModules();
      frc2::CommandPtr StopCommand();
    //SmartDashboard
      frc2::CommandPtr SmartDashboardOutputCommand();

    //Odometry
        
      //void UpdateOdometry(frc::Pose2d m_pose);

      /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
      frc::Pose2d GetPosition();

        /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
      void ResetPosition(frc::Pose2d pose);
      void SetPositionToZeroDistance();

      frc2::CommandPtr Allign(Direction direction);


 private:
    //Gyro
    frc::ADIS16470_IMU m_gyro;

    DriveModule m_frontLeftModule{kFrontLeftSpeedMotorID, kFrontLeftDirectionMotorID, 
                                   kFrontLeftDirectionEncoderID, kFrontLeftMagOffset, "Front Left"};
    DriveModule m_frontRightModule{kFrontRightSpeedMotorID, kFrontRightDirectionMotorID, 
                                   kFrontRightDirectionEncoderID, kFrontRightMagOffset, "Front Right"};
    DriveModule m_backLeftModule{kBackLeftSpeedMotorID, kBackLeftDirectionMotorID, 
                                   kBackLeftDirectionEncoderID, kBackLeftMagOffset, "Back Left"};
    DriveModule m_backRightModule{kBackRightSpeedMotorID, kBackRightDirectionMotorID, 
                                   kBackRightDirectionEncoderID, kBackRightMagOffset, "Back Right"};
    
    frc::SwerveDriveKinematics<4> kinematics{kFrontLeftLocation, 
                                            kFrontRightLocation, 
                                            kBackLeftLocation,
                                            kBackRightLocation};

    
    nt::StructArrayPublisher<frc::SwerveModuleState> m_statePublisher; 
    nt::StructPublisher<frc::Rotation2d> m_headingPublisher; 

    // PoseEstimator class for tracking robot pose
    // 4 defines the number of modules
    frc::SwerveDrivePoseEstimator<4> m_poseEstimator{kinematics, frc::Rotation2d{GetHeading()},
                                {m_frontLeftModule.GetPosition(), m_frontRightModule.GetPosition(),
                                  m_backLeftModule.GetPosition(), m_backRightModule.GetPosition()}, 
                                   frc::Pose2d{}, 
                                  // ( double 0.5, double 0.5, units::radian_t(1)), 
                                  // ( double 0.5, double 0.5, units::radian_t(1))
                                  };
                                            
  public:
  frc::SlewRateLimiter<units::meters_per_second> m_xLimiter{kSlewRateTranslation};
  frc::SlewRateLimiter<units::meters_per_second> m_yLimiter{kSlewRateTranslation};
  frc::SlewRateLimiter<units::radians_per_second> m_rotLimiter{kSlewRateRotation};
};