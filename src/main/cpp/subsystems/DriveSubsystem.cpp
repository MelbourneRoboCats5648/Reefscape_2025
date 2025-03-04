#include "subsystems/DriveSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace DriveConstants;
using namespace CAN_Constants;
using namespace ctre::phoenix6;

DriveSubsystem::DriveSubsystem()
  : m_gyro(kGyroDeviceID, kCanId)
   {
  m_statePublisher = nt::NetworkTableInstance::GetDefault()
      .GetStructArrayTopic<frc::SwerveModuleState>("DriveTrain/SwerveStates").Publish();
  m_headingPublisher = nt::NetworkTableInstance::GetDefault()
      .GetStructTopic<frc::Rotation2d>("DriveTrain/Heading").Publish();
  m_fieldHeadingPublisher = nt::NetworkTableInstance::GetDefault()
      .GetStructTopic<frc::Rotation2d>("DriveTrain/FieldHeading").Publish();

  /* Configure Pigeon2 */
  configs::Pigeon2Configuration toApply{};

  // FIXME - might need to configure some pigeon parameters here
  /* User can change the configs if they want, or leave it empty for factory-default */

  m_gyro.GetConfigurator().Apply(toApply);

  /* Speed up signals to an appropriate rate */
  BaseStatusSignal::SetUpdateFrequencyForAll(100_Hz, m_gyro.GetYaw(), m_gyro.GetGravityVectorZ()); // ISSUE 90: IDK if this is NEEDED
  /**
   * When we teleop init, set the yaw of the Pigeon2 and wait for the setter to take affect.
   */
  m_gyro.SetYaw(DriveConstants::initialGyroAngle, 100_ms); // Set to initial yaw angle and wait up to 100 milliseconds for the setter to take affect
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
    m_fieldHeadingPublisher.Set(
      GetFieldHeading()
    );
}

void DriveSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

void DriveSubsystem::ResetGyro() {
  m_gyro.Reset();
}

frc2::CommandPtr DriveSubsystem::ResetGyroCommand() {
  return RunOnce([this] { ResetGyro(); });
}

void DriveSubsystem::ResetFieldGyroOffset() {
  m_fieldGyroOffset = GetHeading();
}

frc2::CommandPtr DriveSubsystem::ResetFieldGyroOffsetCommand() {
  return RunOnce([this] { ResetFieldGyroOffset(); });
}

units::degree_t DriveSubsystem::GetHeading() const {
  return m_gyro.GetRotation2d().Degrees(); // GetYaw() didn't work for some reason
}

units::degree_t DriveSubsystem::GetFieldHeading() const {
  return GetHeading() - m_fieldGyroOffset;
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           units::second_t period) {
  Drive(xSpeed, ySpeed, rot, m_fieldRelative, period);
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot, bool fieldRelative,
                           units::second_t period) {
   auto states =
       kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
           fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                               xSpeed, ySpeed, rot, frc::Rotation2d{GetFieldHeading()})
                         : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
           period));

  kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeftModule.SetModule(fl);
  m_frontRightModule.SetModule(fr);
  m_backLeftModule.SetModule(bl);
  m_backRightModule.SetModule(br);
}

frc2::CommandPtr DriveSubsystem::SwitchRobotRelativeCommand() {
  return RunOnce([this] { m_fieldRelative = false; });
}

frc2::CommandPtr DriveSubsystem::SwitchFieldRelativeCommand() {
  return RunOnce([this] { m_fieldRelative = true; });
}

frc2::CommandPtr DriveSubsystem::ToggleFieldRelativeCommand() {
  return RunOnce([this] { m_fieldRelative = !m_fieldRelative; });
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

