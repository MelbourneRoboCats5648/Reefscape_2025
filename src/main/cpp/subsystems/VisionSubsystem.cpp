#include "subsystems/VisionSubsystem.h"


VisionSubsystem::VisionSubsystem() {
  // Implementation of subsystem constructor goes here.
}

void VisionSubsystem::GetAprilTagID(){
    double apriltagID = LimelightHelpers::getFiducialID();
    std::cout << apriltagID;
}
void VisionSubsystem::GetTagTY(){

    double ty = LimelightHelpers::getTY("");
    std::cout << ty;
}


void VisionSubsystem::GetTagTX(){

    double tx = LimelightHelpers::getTX("");
    std::cout << tx;
}

void VisionSubsystem::Periodic() {
  //m_aprilTagLoop.Poll();
  
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

