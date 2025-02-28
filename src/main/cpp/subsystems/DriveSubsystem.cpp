#include "subsystems/DriveSubsystem.h"

//PathPlan Stuff
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>

using namespace DriveConstants;
using namespace pathplanner;

DriveSubsystem::DriveSubsystem()   
    : m_poseEstimator{kinematics,
      frc::Rotation2d{GetHeading()},
      {m_frontLeftModule.GetPosition(), m_frontRightModule.GetPosition(),
       m_backLeftModule.GetPosition(), m_backRightModule.GetPosition()},
       frc::Pose2d{}}
{
  m_gyro.Calibrate();
  
  m_statePublisher = nt::NetworkTableInstance::GetDefault()
      .GetStructArrayTopic<frc::SwerveModuleState>("/SwerveStates").Publish();
  m_headingPublisher = nt::NetworkTableInstance::GetDefault()
      .GetStructTopic<frc::Rotation2d>("/DriveHeading").Publish();

  //PathPlanner
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = RobotConfig::fromGUISettings();

    // Configure the AutoBuilder last
    AutoBuilder::configure(
        [this](){ return GetPosition(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ResetPosition(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds /*,auto feedforwards*/){ 
                Drive(speeds.vx, speeds.vy, speeds.omega, false); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  
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

  m_poseEstimator.Update(frc::Rotation2d{GetHeading()}, //idk if this should be m_odometry and m_poseestimator https://github.com/LimelightVision/limelight-examples/blob/main/java-wpilib/swerve-megatag-odometry/src/main/java/frc/robot/Drivetrain.java#L88
      {m_frontLeftModule.GetPosition(), m_frontRightModule.GetPosition(),
       m_backLeftModule.GetPosition(), m_backRightModule.GetPosition()});

        //copied from https://github.com/LimelightVision/limelight-examples/blob/main/java-wpilib/swerve-megatag-odometry/src/main/java/frc/robot/Drivetrain.java#L87
       bool useMegaTag2 = true;
       bool doRejectUpdate = false;
         if(useMegaTag2 == false)
          {
            LimelightHelpers::PoseEstimate mt1 = LimelightHelpers::getBotPoseEstimate_wpiBlue("limelight");
            
            if(mt1.tagCount == 1 && sizeof(mt1.rawFiducials) == 1)
            {
              if(mt1.rawFiducials[0].ambiguity > .7)
              {
                doRejectUpdate = true;
              }
              if(mt1.rawFiducials[0].distToCamera > 3)
              {
                doRejectUpdate = true;
              }
            }
            if(mt1.tagCount == 0)
            {
              doRejectUpdate = true;
            }

            if(!doRejectUpdate)
            {
              const wpi::array<double, 3> stdInput = {0.5, 0.5, 99999999.0};
              m_poseEstimator.SetVisionMeasurementStdDevs(stdInput);

              m_poseEstimator.AddVisionMeasurement(
                  mt1.pose,
                  mt1.timestampSeconds);
            }
          }
        else if (useMegaTag2 == true)
        {
          LimelightHelpers::SetRobotOrientation("limelight", (double) m_poseEstimator.GetEstimatedPosition().Rotation().Degrees() , 0, 0, 0, 0, 0);
          LimelightHelpers::PoseEstimate mt2 = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
          if(units::math::abs(m_gyro.GetRate()) > units::angular_velocity::degrees_per_second_t(720)) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
          {
            doRejectUpdate = true;
          }
          if(mt2.tagCount == 0)
          {
            doRejectUpdate = true;
          }
          if(!doRejectUpdate)
          {
              const wpi::array<double, 3> stdInput = {0.7, 0.7, 99999999.0};
              m_poseEstimator.SetVisionMeasurementStdDevs(stdInput);
            m_poseEstimator.AddVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
          }
        }
  
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
   
  /* 
    drive inputs > chassis speed (depending on field relative)
    store chassis speed in private member variable;
    convert the chassis speed to states (discritize etc.)
  */

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

frc::ChassisSpeeds DriveSubsystem::GetRobotRelativeSpeeds() {
  auto fl = m_frontLeftModule.GetState();
  auto fr = m_frontRightModule.GetState();
  auto bl = m_backLeftModule.GetState();
  auto br = m_backRightModule.GetState();
  return kinematics.ToChassisSpeeds(fl, fr, bl, br);
}

//Stops
void DriveSubsystem::StopAllModules() {

  m_frontLeftModule.StopMotors();
  m_frontRightModule.StopMotors();
  m_backLeftModule.StopMotors();
  m_backRightModule.StopMotors();
}

frc2::CommandPtr DriveSubsystem::StopCommand() {
  return Run([this] {StopAllModules(); });
}


//Pose Estimation
frc::Pose2d DriveSubsystem::GetPosition() {
  return m_poseEstimator.GetEstimatedPosition();
}

void DriveSubsystem::ResetPosition(frc::Pose2d pose) {
  m_poseEstimator.ResetPosition(frc::Rotation2d{GetHeading()},
                  {m_frontLeftModule.GetPosition(), m_frontRightModule.GetPosition(),
                  m_backLeftModule.GetPosition(), m_backRightModule.GetPosition()}, 
                  pose);
}

//Reset Encoders               
void DriveSubsystem::SetPositionToZeroDistance() {
  m_frontLeftModule.SetModulePositionToZeroDistance();
  m_frontRightModule.SetModulePositionToZeroDistance();
  m_backLeftModule.SetModulePositionToZeroDistance();
  m_backRightModule.SetModulePositionToZeroDistance();
}   

//SmartDashboard
frc2::CommandPtr DriveSubsystem::SmartDashboardOutputCommand() {
return Run([this] {m_frontLeftModule.OutputPositionToDashboard();
                   m_frontRightModule.OutputPositionToDashboard();
                   m_backLeftModule.OutputPositionToDashboard();
                   m_backRightModule.OutputPositionToDashboard(); });
}

//Align
frc2::CommandPtr DriveSubsystem::Allign(Direction direction) {

}
