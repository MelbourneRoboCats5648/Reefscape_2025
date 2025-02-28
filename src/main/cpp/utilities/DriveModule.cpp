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

    TalonFXConfiguration directionMotorConfig;
    directionMotorConfig.Feedback.SensorToMechanismRatio = 150.0/7.0;
    directionMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;
    directionMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    directionMotorConfig.CurrentLimits.SupplyCurrentLimit = 40_A;
    directionMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30_A;
    directionMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1_s;
    directionMotorConfig.MotorOutput.Inverted = true;  // +V should rotate the motor counter-clockwise
    directionMotorConfig.MotorOutput.NeutralMode = NeutralModeValue::Brake;

    // direction motor configuration
    m_directionMotor.GetConfigurator().Apply(directionMotorConfig);

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

void DriveModule::OutputPositionToDashboard(){
  frc::SmartDashboard::PutNumber(m_name, m_directionEncoder.GetAbsolutePosition().GetValueAsDouble());
}

void DriveModule::SetModule(frc::SwerveModuleState state) {
  // encoder range -0.5 +0.5,  GetValue returns rotation
  // encoder current angle is -pi to +pi
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

  m_directionMotor.SetVoltage(units::voltage::volt_t{turnOutput}); 
}

frc::SwerveModuleState DriveModule::GetState() {
  return frc::SwerveModuleState{
    m_speedMotor.GetVelocity().GetValueAsDouble()*kWheelCircumference/1_s, //metres per sec
    m_directionEncoder.GetAbsolutePosition().GetValue()
  };
}



