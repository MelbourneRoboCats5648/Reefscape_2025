#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <units/angle.h>

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

#include <ctre/phoenix6/Pigeon2.hpp>

using namespace ctre::phoenix6::hardware;

using namespace DriveConstants;
using namespace OperatorConstants;
using namespace CAN_Constants;


class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

    void Periodic() override;
    void SimulationPeriodic() override;

    void ResetGyro();
    frc2::CommandPtr ResetGyroCommand();

    units::degree_t  GetHeading() const;
    units::degree_t  GetFieldHeading() const;

    void Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           units::second_t period = DriveConstants::kDrivePeriod);
    
    void Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot, bool fieldRelative,
                           units::second_t period = DriveConstants::kDrivePeriod);

    void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);
    void StopAllModules();
    frc2::CommandPtr StopCommand();
    frc2::CommandPtr SmartDashboardOutputCommand();

    bool m_fieldRelative = false; // robot-centric by default
    frc2::CommandPtr SwitchRobotRelativeCommand();
    frc2::CommandPtr SwitchFieldRelativeCommand();
    frc2::CommandPtr ToggleFieldRelativeCommand();

 private:
    //Gyro
    Pigeon2 m_gyro{kGyroDeviceID, kCanId};


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
    nt::StructPublisher<frc::Rotation2d> m_fieldHeadingPublisher;

    units::degree_t m_fieldGyroOffset = 0.0_deg;
                                            
  public:
  frc::SlewRateLimiter<units::meters_per_second> m_xLimiter{kSlewRateTranslation};
  frc::SlewRateLimiter<units::meters_per_second> m_yLimiter{kSlewRateTranslation};
  frc::SlewRateLimiter<units::radians_per_second> m_rotLimiter{kSlewRateRotation};
};