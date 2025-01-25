#include "subsystems/VisionSubsystem.h"


VisionSubsystem::VisionSubsystem() {
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr VisionSubsystem::GetAprilTagID(){
    return Run([this] {double apriltagID = LimelightHelpers::getFiducialID();
                                                    std::cout << apriltagID; })
         .FinallyDo([this]{std::cout <<"command stopped";
         });
}
frc2::CommandPtr VisionSubsystem::GetTagTY(){
    return Run([this] {double ty = LimelightHelpers::getTY("");
                                            std::cout << ty; })
         .FinallyDo([this]{std::cout <<"command stopped";
         });
}
    


frc2::CommandPtr VisionSubsystem::GetTagTX(){
    return Run([this] {double tx = LimelightHelpers::getTX("");
                                                std::cout << tx;})
         .FinallyDo([this]{std::cout <<"command stopped";
         });

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

