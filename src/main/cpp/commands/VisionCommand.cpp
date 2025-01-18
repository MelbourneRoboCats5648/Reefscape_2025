#include "commands/VisionCommand.h"

namespace cmd {
using namespace frc2::cmd;

VisionCommand::VisionCommand(VisionSubsystem* visionSubsystem
                                                            /*, DriveSubsystem* driveSubsytem*/)
    : m_visionSubsystem(visionSubsystem)
      //m_driveSubsystem{driveSubsystem}
    
{
  // Register that this command requires the subsystem.
  AddRequirements(m_visionSubsystem
                 /*,m_driveSubsystem*/);

}



frc2::CommandPtr VisionCommand::AimRobotToTargetReef(){
    return Run([this] { 
        
        double targettingAngularVelocity = 1.0;//(//m_visionSubsystem.GetInstance().GetTargetingAngularVelocityReef() *= 1.0
                                            //*= m_driveTrain.kMaxAngularSpeed
                                            //);
        double targettingFowardSpeed = 1.0;//(m_visionSubsystem->GetTargetingForwardSpeedReef() *= 1.0
                                            //*= m_driveTrain.kMaxFowardSpeed
                                            //);
        return targettingAngularVelocity;
        return targettingFowardSpeed;

    }).AndThen
        ([this]{

        });
    }

    
} //cmd namespace
    