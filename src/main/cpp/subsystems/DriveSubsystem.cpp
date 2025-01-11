#include "subsystems/DriveSubsystem.h"

DriveSubsystem::DriveSubsystem() {
  m_gyro.Calibrate();
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void DriveSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

// frc2::CommandPtr ExampleSubsystem::ExampleMethodCommand() {
//   // Inline construction of command goes here.
//   // Subsystem::RunOnce implicitly requires `this` subsystem.
//   return RunOnce([/* this */] { /* one-time action goes here */ });
// }

void DriveSubsystem::Drive(frc::ChassisSpeeds chassisSpeed) {
//   auto states =
//       kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
//           fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
//                               xSpeed, ySpeed, rot, GetHeading())
//                         : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
//           period));

    auto states = kinematics.ToSwerveModuleStates(chassisSpeed);

    const units::meters_per_second_t kMaxSpeedMPS = 5.0_mps;
    kinematics.DesaturateWheelSpeeds(&states, kMaxSpeedMPS);

  auto [fl, fr, bl, br] = states;

  m_frontLeftModule.SetModule(fl);
  m_frontRightModule.SetModule(fr);
  m_backLeftModule.SetModule(bl);
  m_backRightModule.SetModule(br);
}

//The following stuff I've copied over from crescendo and am unsure about the necessity

void DriveSubsystem::StopAllModules()
{
  m_frontLeftModule.StopMotors();
  m_frontRightModule.StopMotors();
  m_backLeftModule.StopMotors();
  m_backRightModule.StopMotors();
}

double DriveSubsystem::GetPositionDistance()
{
  return (m_frontLeftModule.GetModulePositionDistance()+
  m_frontRightModule.GetModulePositionDistance()+
  m_backLeftModule.GetModulePositionDistance()+
  m_backRightModule.GetModulePositionDistance())/4;
}

void DriveSubsystem::SetPositionToZeroDistance()
{
  m_frontLeftModule.SetModulePositionToZeroDistance();
  m_frontRightModule.SetModulePositionToZeroDistance();
  m_backLeftModule.SetModulePositionToZeroDistance();
  m_backRightModule.SetModulePositionToZeroDistance();
}

