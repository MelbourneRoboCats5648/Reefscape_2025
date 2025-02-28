#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem() {
  m_gyro.Calibrate();
  m_statePublisher = nt::NetworkTableInstance::GetDefault()
      .GetStructArrayTopic<frc::SwerveModuleState>("DriveTrain/SwerveStates").Publish();
  m_headingPublisher = nt::NetworkTableInstance::GetDefault()
      .GetStructTopic<frc::Rotation2d>("DriveTrain/Heading").Publish();
  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});
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

frc::Pose2d DriveSubsystem::getPose(){
  return frc::Pose2d{}; //Issue 76 replace this with actual pose function
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

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kinematics.DesaturateWheelSpeeds(&desiredStates,
                                         DriveConstants::kMaxSpeed);
  m_frontLeftModule.SetModule(desiredStates[0]);
  m_frontRightModule.SetModule(desiredStates[1]);
  m_backLeftModule.SetModule(desiredStates[2]);
  m_backRightModule.SetModule(desiredStates[3]);
}

void DriveSubsystem::StopAllModules() {
  m_frontLeftModule.StopMotors();
  m_frontRightModule.StopMotors();
  m_backLeftModule.StopMotors();
  m_backRightModule.StopMotors();
}

frc2::CommandPtr DriveSubsystem::StopCommand() {
  return Run([this] {StopAllModules(); });
}

frc2::CommandPtr DriveSubsystem::SmartDashboardOutputCommand() {
  return Run([this] {m_frontLeftModule.OutputPositionToDashboard();
                   m_frontRightModule.OutputPositionToDashboard();
                   m_backLeftModule.OutputPositionToDashboard();
                   m_backRightModule.OutputPositionToDashboard(); });
}

//Calculate Trajectory
frc::Trajectory DriveSubsystem::CalculateTrajectory(units::meter_t deltaX, units::meter_t deltaY, units::turn_t deltaTheta)
{
   frc::TrajectoryConfig config(kMaxSpeed, kMaxAcceleration); //Issue 76 these Max constants need to be diff from general drive ones
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(kinematics);
  // An example trajectory to follow.  All units in meters.
  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    {frc::Pose2d{0_m, 0_m, 0_tr}, frc::Pose2d{deltaX, deltaY, deltaTheta}}, //Issue 76 this will actually be DriveSubsystem.GetPosition();
      // Pass the config
      config);

  return trajectory;
}



