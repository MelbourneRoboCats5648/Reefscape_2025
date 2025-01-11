// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/VictorSP.h>

//Port Numbers
const int kMotorIntakeArmPort = 2;
const int kMotorIntakeWheelPort = 3;

//Motor Speed Constants
const double intakeArmRetractSpeed = 0.4;
const double intakeArmExtendSpeed = -0.4;

const double intakeWheelInSpeed = 0.6;
const double intakeWheelOutSpeed = -7.0;


class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  /**
   * Example command factory method.
   */
  frc2::CommandPtr CollectCommand();
  frc2::CommandPtr EjectCommand();
  frc2::CommandPtr RetractCommand();
  frc2::CommandPtr ExtendCommand();



  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
    frc::VictorSP m_motorIntakeWheel{kMotorIntakeWheelPort};
    frc::VictorSP m_motorIntakeArm{kMotorIntakeArmPort};
};
