// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Actions;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//Constants
import frc.robot.Constants.Motors;
//Commands
import frc.robot.commands.ManualFunctions.CageLift.cageReset;

public class LiftServo extends SubsystemBase {
  public LiftServo() {
  setDefaultCommand(new cageReset(this));
  }

  @Override
  public void periodic() {
  }

  //Commands 
  public void cageRelease(){
    Motors.m_release.setAngle(0);
  }
  public void cageReset(){
    Motors.m_release.setAngle(110);
}

}
