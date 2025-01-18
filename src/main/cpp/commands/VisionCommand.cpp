#include "commands/VisionCommand.h"
/*
namespace cmd {
using namespace frc2::cmd;

VisionCommand::VisionCommand(VisionSubsystem* visionSubsystem,
                             DriveSubsystem* driveSubsystem)
    : m_visionSubsystem(visionSubsystem),
      m_driveSubsystem(driveSubsystem)
    
{
  // Register that this command requires the subsystem.
  AddRequirements(m_visionSubsystem,
                  m_driveSubsystem);

}

double m_targettingAngularVelocity(
    m_visionSubsystem.GetTargettingAngularVelocityReef() *= m_driveTrain.
)

double m_targettingForwardSpeed()
    


frc2::CommandPtr VisionCommand::AimRobotToTargetReef(){
    return Run([this] { 
        
        double m_visionSubsystem->m_targettingAngularVelocity = m_visionSubsystem->GetTargetingAngularVelocityReef() *= 1.0;
                                            //*= m_driveTrain.kMaxAngularSpeed
                                            //);
        double targettingFowardSpeed = 1.0;//(m_visionSubsystem->GetTargetingForwardSpeedReef() *= 1.0
                                            //*= m_driveTrain.kMaxFowardSpeed
                                            //);
        return targettingAngularVelocity;
        return targettingFowardSpeed;

    }).AndThen
        ([this, m_visionSubsystem]{
            m_visionSubsystem->m_targettingAngularVelocity

        });
    }

    
} //cmd namespace

/*

frc2::CommandPtr VisionCommand::AimRobotToTargetReef(){
    return Run([this, m_visionSubsystem] { 
        
        double m_visionSubsystem->m_targettingAngularVelocity = m_visionSubsystem->GetTargetingAngularVelocityReef() *= 1.0;
                                            //*= m_driveTrain.kMaxAngularSpeed
                                            //);
        double targettingFowardSpeed = 1.0;//(m_visionSubsystem->GetTargetingForwardSpeedReef() *= 1.0
                                            //*= m_driveTrain.kMaxFowardSpeed
                                            //);
        return targettingAngularVelocity;
        return targettingFowardSpeed;

    }).AndThen
        ([this, m_visionSubsystem]{
            m_visionSubsystem->m_targettingAngularVelocity

        });
    }

    
} //cmd namespace

*/
