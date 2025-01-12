#include "utilities/DriveModule.h"
#include <iostream>

using namespace ctre::phoenix6::signals;
using namespace ctre::phoenix6::configs;

DriveModule::DriveModule(int speedMotorID, int directionMotorID, int directionEncoderID, double magOffset, std::string name)
            :m_speedMotor(speedMotorID, "rio"),
            m_directionMotor(directionMotorID, "rio"),
            m_directionEncoder(directionEncoderID, "rio"),
            m_magOffset(magOffset),
            m_name(name),
            m_turningPIDController{
                1,
                0.0,
                0.0,
                {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}}

    {

    // Config CANCoder   
    CANcoderConfiguration cancoderConfig;
    //cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue::Signed_PlusMinusHalf; //fixme
    
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue::CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = units::angle::turn_t(m_magOffset); //fix me
    m_directionEncoder.GetConfigurator().Apply(cancoderConfig);

    // Drive motor config
    // To setup the drive motor to be able to drive at a target speed
    TalonFXConfiguration speedMotorConfig;
    speedMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::RotorSensor;
    speedMotorConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
    speedMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;
    speedMotorConfig.Slot0.kP = 0.3;
    speedMotorConfig.Slot0.kI = 0.0;
    speedMotorConfig.Slot0.kD = 0.0;
    speedMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    speedMotorConfig.Slot0.kV = 0.1;
    //speedMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;      // Amps //fix me
    //speedMotorConfig.CurrentLimits.SupplyCurrentThreshold = 60;  // Amps //fix me
    //speedMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;    // Seconds //fix me
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
  // encoder current angle is -pi to +pi
  units::angle::radian_t encoderCurrentAngleRadians = 
         units::angle::radian_t{m_directionEncoder.GetAbsolutePosition().GetValueAsDouble()*2*M_PI};
    // updates state variable angle to the optimum change in angle
  auto optimizedState = state = frc::SwerveModuleState::Optimize(state, encoderCurrentAngleRadians);

  // Setting Motor Speed
  ////const units::meters_per_second_t MAX_SPEED_MPS = 5.0_mps;
  // double normalisedSpeed = state.speed / MAX_SPEED_MPS; // max speed set in DriveTrain::SetAllModules()

  //to find desired wheel speed:
  // divide desired robot speed (m/s) by wheel circumference (set CIRCUMFERECNE in header file)
  
  // to set the speed using control onboard the motor, use m_speedMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{desiredWheelSpeed}); // desiredWheelSpeed in turns per second
  units::angular_velocity::turns_per_second_t desiredWheelSpeed{(state.speed.value())/WHEEL_CIRCUMFERENCE.value()};
  m_speedMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{desiredWheelSpeed*6.75});
  ////m_speedMotor.Set(normalisedSpeed); 

  // Calculate the turning motor output from the turning PID controller. 
  const auto turnOutput = m_turningPIDController.Calculate(
    encoderCurrentAngleRadians, optimizedState.angle.Radians());

  m_directionMotor.SetVoltage(units::voltage::volt_t{-1.0 * turnOutput});
}

double DriveModule::GetModulePositionDistance()
{
  double distance = m_speedMotor.GetPosition().GetValueAsDouble()*0.3198;  
  std::cout << m_name << " modulePosition " << distance << std::endl;
  return distance;
}

void DriveModule::SetModulePositionToZeroDistance()
{
  m_speedMotor.SetPosition(units::angle::turn_t {0.0});
}







