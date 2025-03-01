#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/core/CoreCANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <units/angle.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/controller/ProfiledPIDController.h>

#include "Constants.h"

using namespace ctre::phoenix6::hardware;


class DriveModule{
public:
    DriveModule(int speedMotorID, 
                int directionMotorID, 
                int directionEncoderID, 
                units::angle::turn_t magOffset, 
                std::string name);

public:
    // sets the drive of all motors to zero    
    void StopMotors();
    void SetModule(frc::SwerveModuleState state);
    void OutputPositionToDashboard();
    frc::SwerveModuleState GetState();

private:
    void SetSpeedMotorConfig();
    void SetDirectionEncoderConfig();
    void SetDirectionMotorConfig();
    
private:
    TalonFX m_speedMotor;
    TalonFX m_directionMotor;
    CANcoder m_directionEncoder;
    units::angle::turn_t m_magOffset;
    std::string m_name; 
    frc::ProfiledPIDController<units::radians> m_turningPIDController;

};