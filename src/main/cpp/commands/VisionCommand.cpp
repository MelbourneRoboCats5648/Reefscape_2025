#include "commands/VisionCommand.h"

VisionCommand::VisionCommand(VisionSubsystem* visionSubsystem)
    : m_visionSubsystem{visionSubsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_visionSubsystem);
}

/*
frc2::CommandPtr VisionSubsystem::AimRobotToTargetReef(){
    return Run([this] { 
        
        double tx = LimelightHelpers::getTX("");
        double ty = LimelightHelpers::getTY("");
        double targettingAngularVelocity = tx * kP;

        targettingAungularVelocity *= Drivetrain.kMaxAngularSpeed; //need to add the drivetrain code for this
        
    
    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
    
        
        double kP = .1;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
        targetingForwardSpeed *= Drivetrain.kMaxSpeed;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
        
     })

   .AndThen
    ([this]{
        //then replace xspeed with limelight with targetting forward speed and 
        //rotation with targetting angilar velocity
        //within drive train
    }); 
    */

    