#include "subsystems/VisionSubsystem.h"


VisionSubsystem::VisionSubsystem() {
  // Implementation of subsystem constructor goes here.
}

void VisionSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  double aprilTagID = LimelightHelpers::getFiducialID();
  frc2::Trigger aprilTagIDTrigger(aprilTagID = 1)
}

void VisionSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}


frc2::CommandPtr VisionSubsystem::AimRobotToTargetReef(){
    return Run([this] { 
        /*
        double tx = LimelightHelpers::getTX("");
        double ty = LimelightHelpers::getTY("");
        double targettingAngularVelocity = tx * kP;

        targettingAungularVelocity *= Drivetrain.kMaxAngularSpeed; //need to add the drivetrain code for this
        */
    /*
    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
    */
        /*
        double kP = .1;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
        targetingForwardSpeed *= Drivetrain.kMaxSpeed;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
        */
     })

   .AndThen
    ([this]{
        //then replace xspeed with limelight with targetting forward speed and 
        //rotation with targetting angilar velocity
        //within drive train
    }); 


}

