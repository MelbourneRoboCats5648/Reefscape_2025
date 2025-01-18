#include "subsystems/VisionSubsystem.h"


VisionSubsystem::VisionSubsystem() 
  : m_aprilTagLoop(),
    m_aprilTagIDBoolean(&m_aprilTagLoop,
            [this] { return (LimelightHelpers::getFiducialID() == 1.0); }),
    m_aprilTagIDTrigger(m_aprilTagIDBoolean.CastTo<frc2::Trigger>())
{
  // Implementation of subsystem constructor goes here.
  //m_aprilTagIDTrigger = m_aprilTagIDBoolean.CastTo<frc2::Trigger>();
}

void VisionSubsystem::Periodic() {
  m_aprilTagLoop.Poll();
  
  // Implementation of subsystem periodic method goes here.

  //    frc::BooleanEvent m_aprilTagIDBoolean =
  //      frc::BooleanEvent(
  //          &m_aprilTagLoop,
  //          [this] { return (LimelightHelpers::getFiducialID() == 1.0); }); //fixme - floating point equality

   //  m_aprilTagIDTrigger = m_aprilTagIDBoolean.CastTo<frc2::Trigger>();
            
}

void VisionSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}


//frc2::CommandPtr VisionSubsystem::GetTXAndTY(){
    //return Run([this] { 
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
   //  })

   //.AndThen
   // ([this]{
        //then replace xspeed with limelight with targetting forward speed and 
        //rotation with targetting angilar velocity
        //within drive train
   // }); 


//}

frc2::CommandPtr VisionSubsystem::GetTargetingAngularVelocityReef(){
    return Run([this] {
    double tx = LimelightHelpers::getTX("");
    double targettingAngularVelocity = tx * kReefAngularKP;
    //invert since tx is positive when the target is to the right of the crosshair
    //targetingAngularVelocity *= -1.0;
    });
}

frc2::CommandPtr VisionSubsystem::GetTargetingForwardSpeedReef(){
    return Run([this] {
    double ty = LimelightHelpers::getTY("");
    double targettingForwardSpeed = ty * kReefForwardKP;
     //targetingForwardSpeed *= -1.0;
    });
}


