#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/core/CoreCANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <units/angle.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/controller/ProfiledPIDController.h>

#include "constants.h"

using namespace ctre::phoenix6::hardware;

//Module constants defining
 static constexpr auto kModuleMaxAngularVelocity =
      std::numbers::pi * 4_rad_per_s;  // radians per second

 static constexpr auto kModuleMaxAngularAcceleration =
    std::numbers::pi * 8_rad_per_s / 1_s;  // radians per second^2

class DriveModule{
public:
    DriveModule(int speedMotorID, int directionMotorID, int directionEncoderID, double magOffset, std::string name);

public:
    // sets the drive of all motors to zero    
    void StopMotors();
    void SetModule(frc::SwerveModuleState state);
    void OutputPositionToDashboard();

private:
    TalonFX m_speedMotor;
    TalonFX m_directionMotor;
    CANcoder m_directionEncoder;
    double m_magOffset;
    std::string m_name; 
    frc::ProfiledPIDController<units::radians> m_turningPIDController;
    //const units::meter_t WHEEL_RADIUS = 0.0508_m;
    // CIRCUMFERECNE calc here
private:
    // Mechanical Constants
    static constexpr double TURNING_GEAR_RATIO = 150.0 / 7.0;
    static constexpr double DRIVE_GEAR_RATIO = 6.75;  // L2 - Fast kit check the gear ratio
    static constexpr units::meter_t WHEEL_RADIUS = 0.0508_m;
    static constexpr units::meter_t WHEEL_CIRCUMFERENCE = 2 * std::numbers::pi * WHEEL_RADIUS;
};