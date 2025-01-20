#include "subsystems/DriveSubsystem.h"

DriveSubsystem::DriveSubsystem() 
  : m_odometry{kinematics,
      frc::Rotation2d{GetHeading()},
      {m_frontLeftModule.GetPosition(), m_frontRightModule.GetPosition(),
       m_backLeftModule.GetPosition(), m_backRightModule.GetPosition()},
       frc::Pose2d{}}
{
  m_gyro.Calibrate();
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes herSe.
  m_odometry.Update(frc::Rotation2d{GetHeading()},
      {m_frontLeftModule.GetPosition(), m_frontRightModule.GetPosition(),
       m_backLeftModule.GetPosition(), m_backRightModule.GetPosition()});
}

void DriveSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

//Gyro
void DriveSubsystem::ResetGyro() {
  m_gyro.Reset();
}

units::degree_t DriveSubsystem::GetHeading() const {
  return m_gyro.GetAngle();
}

//Drive
void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot, bool fieldRelative,
                           units::second_t period) {
   auto states =
       kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
           fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                               xSpeed, ySpeed, rot, frc::Rotation2d{GetHeading()})
                         : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
           period));

  kinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeftModule.SetModule(fl);
  m_frontRightModule.SetModule(fr);
  m_backLeftModule.SetModule(bl);
  m_backRightModule.SetModule(br);
}


void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kinematics.DesaturateWheelSpeeds(&desiredStates,
                                         DriveConstants::kMaxSpeed);
  m_frontLeftModule.SetModule(desiredStates[0]);
  m_frontRightModule.SetModule(desiredStates[1]);
  m_backLeftModule.SetModule(desiredStates[2]);
  m_backRightModule.SetModule(desiredStates[3]);
}

//Stops
void DriveSubsystem::StopAllModules()
{
  m_frontLeftModule.StopMotors();
  m_frontRightModule.StopMotors();
  m_backLeftModule.StopMotors();
  m_backRightModule.StopMotors();
}

frc2::CommandPtr DriveSubsystem::StopCommand() 
{
return Run([this] {m_frontLeftModule.StopMotors();
                   m_frontRightModule.StopMotors();
                   m_backLeftModule.StopMotors();
                   m_backRightModule.StopMotors(); });

}

//Odometry
frc::Pose2d DriveSubsystem::GetPose() {
  return m_odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
                  frc::Rotation2d{GetHeading()},
                  {m_frontLeftModule.GetPosition(), m_frontRightModule.GetPosition(),
                  m_backLeftModule.GetPosition(), m_backRightModule.GetPosition()}, 
                  pose);
}

//Reset Encoders               
void DriveSubsystem::SetPositionToZeroDistance()
{
  m_frontLeftModule.SetModulePositionToZeroDistance();
  m_frontRightModule.SetModulePositionToZeroDistance();
  m_backLeftModule.SetModulePositionToZeroDistance();
  m_backRightModule.SetModulePositionToZeroDistance();
}   
