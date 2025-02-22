#include "utilities/DriveModule.h"
#include<frc/smartdashboard/SmartDashboard.h>

using namespace ctre::phoenix6::signals;
using namespace ctre::phoenix6::configs;
using namespace DriveConstants;

DriveModule::DriveModule(int speedMotorID, int directionMotorID, int directionEncoderID, units::angle::turn_t magOffset, std::string name)
            :m_speedMotor(speedMotorID, "rio"),
            m_directionMotor(directionMotorID, "rio"),
            m_directionEncoder(directionEncoderID, "rio"),
            m_magOffset(magOffset),
            m_name(name),
            m_turningPIDController{
                kTurnKP,
                kTurnKI,
                kTurnKD,
                {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}}

    {

    m_directionMotor.SetPosition(m_directionEncoder.GetAbsolutePosition().
                                WaitForUpdate(250_ms).GetValue());
                                
    m_directionMotor.SetNeutralMode(NeutralModeValue::Coast);

    // Config CANCoder   
    CANcoderConfiguration cancoderConfig;
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5_tr;
    
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue::CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = m_magOffset;
    m_directionEncoder.GetConfigurator().Apply(cancoderConfig);

    // Drive motor config
    // To setup the drive motor to be able to drive at a target speed
    TalonFXConfiguration speedMotorConfig;
    speedMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::RotorSensor;
    speedMotorConfig.Feedback.SensorToMechanismRatio = kDriveGearRatio;
    speedMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;
    speedMotorConfig.Slot0.kP = kSpeedMotorKP;
    speedMotorConfig.Slot0.kI = kSpeedMotorKI;
    speedMotorConfig.Slot0.kD = kSpeedMotorKD;
    speedMotorConfig.Slot0.kV = kSpeedMotorkV;
    speedMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    speedMotorConfig.CurrentLimits.SupplyCurrentLimit = 50_A;      // Amps
    speedMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 60_A;  // Amps
    speedMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1_s;    // Seconds    
    speedMotorConfig.MotorOutput.Inverted = true;  // +V should rotate the motor counter-clockwise
    speedMotorConfig.MotorOutput.NeutralMode = NeutralModeValue::Brake;
    m_speedMotor.GetConfigurator().Apply(speedMotorConfig);

    m_turningPIDController.EnableContinuousInput(
                units::angle::radian_t{-1.0*M_PI},
                units::angle::radian_t{M_PI});
    }




void DriveModule::StopMotors()
{
  m_directionMotor.Set(0);
  m_speedMotor.Set(0);
}

void DriveModule::SetModule(frc::SwerveModuleState state) {
  // encoder range -0.5 +0.5,  GetValue returns rotation
  // encoder current angle is -pi to +pi (i think this was removed... im not sure so should test again)
  units::angle::radian_t encoderCurrentAngleRadians = 
                          m_directionEncoder.GetAbsolutePosition().GetValue();
    // updates state variable angle to the optimum change in angle
  state.Optimize(encoderCurrentAngleRadians);
  // Scale speed by cosine of angle error. This scales down movement
  // perpendicular to the desired direction of travel that can occur when
  // modules change directions. This results in smoother driving.
  state.CosineScale(encoderCurrentAngleRadians);

  // to set the speed using control onboard the motor, use m_speedMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{desiredWheelSpeed}); // desiredWheelSpeed in turns per second
  units::angular_velocity::turns_per_second_t desiredWheelSpeed{(state.speed.value())/kWheelCircumference.value()};
  m_speedMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{desiredWheelSpeed*kDriveGearRatio});

  // Calculate the turning motor output from the turning PID controller. 
  const auto turnOutput = m_turningPIDController.Calculate(
    encoderCurrentAngleRadians, state.angle.Radians());

  m_directionMotor.SetVoltage(units::voltage::volt_t{1.0 * turnOutput}); 
}

frc::SwerveModulePosition DriveModule::GetPosition() {
  return {units::meter_t{m_speedMotor.GetPosition().GetValueAsDouble()*kWheelCircumference},
          frc::Rotation2d{m_directionEncoder.GetAbsolutePosition().GetValue()}}; //was previously .GetValueAsDouble()*2*M_PI}
}

void DriveModule::SetModulePositionToZeroDistance()
{
  m_speedMotor.SetPosition(units::angle::turn_t {0.0});
}

units::meters_per_second_t DriveModule::GetSpeed() {
  return (m_speedMotor.GetVelocity().GetValue().value() * kWheelCircumference.value()) * 1_mps;
}

frc::Rotation2d DriveModule::GetAngle() {
  units::radian_t turnAngle = m_directionEncoder.GetAbsolutePosition().GetValue() /* 2.0 * M_PI*/;
  return turnAngle;
}


frc::SwerveModuleState DriveModule::GetState() {
  return {GetSpeed(), GetAngle()};
} 

void DriveModule::OutputPositionToDashboard(){
  frc::SmartDashboard::PutNumber(m_name, m_directionEncoder.GetAbsolutePosition().GetValueAsDouble());
}







