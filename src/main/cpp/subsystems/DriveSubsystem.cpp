#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem() {
  m_gyro.Calibrate();
  m_statePublisher = nt::NetworkTableInstance::GetDefault()
      .GetStructArrayTopic<frc::SwerveModuleState>("/SwerveStates").Publish();
  m_headingPublisher = nt::NetworkTableInstance::GetDefault()
      .GetStructTopic<frc::Rotation2d>("/DriveHeading").Publish();
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
   // Periodically send a set of module states
    m_statePublisher.Set(
      std::vector{
        m_frontLeftModule.GetState(),
        m_frontRightModule.GetState(),
        m_backLeftModule.GetState(),
        m_backRightModule.GetState()
      });
    m_headingPublisher.Set(
      GetHeading()
    );
}

void DriveSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

void DriveSubsystem::ResetGyro() {
  m_gyro.Reset();
}

units::degree_t DriveSubsystem::GetHeading() const {
  return m_gyro.GetAngle();
}

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

  kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeftModule.SetModule(fl);
  m_frontRightModule.SetModule(fr);
  m_backLeftModule.SetModule(bl);
  m_backRightModule.SetModule(br);
}
frc2::CommandPtr DriveSubsystem::DriveCommand(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot, bool fieldRelative, units::second_t period) 
{
  return Run ([this, xSpeed, ySpeed, rot, fieldRelative, period ] {
    Drive(xSpeed, ySpeed, rot, fieldRelative, period);})
          .FinallyDo ([this, fieldRelative, period] {
          Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, fieldRelative, period);});

}

void DriveSubsystem::StopAllModules()
{
  m_frontLeftModule.StopMotors();
  m_frontRightModule.StopMotors();
  m_backLeftModule.StopMotors();
  m_backRightModule.StopMotors();
}

frc2::CommandPtr DriveSubsystem::StopCommand() 
{
return Run([this] {StopAllModules(); });
}

frc2::CommandPtr DriveSubsystem::SmartDashboardOutputCommand() 
{
return Run([this] {m_frontLeftModule.OutputPositionToDashboard();
                   m_frontRightModule.OutputPositionToDashboard();
                   m_backLeftModule.OutputPositionToDashboard();
                   m_backRightModule.OutputPositionToDashboard(); });
}

