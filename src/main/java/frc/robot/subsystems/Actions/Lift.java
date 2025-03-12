// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Actions;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//Constants
import frc.robot.Constants.Motors;
//Commands
import frc.robot.commands.ManualFunctions.CageLift.cageStop;
import frc.robot.commands.ManualFunctions.CageLift.cageReset;

public class Lift extends SubsystemBase {
  public boolean released = false;
  public Lift() {
    setDefaultCommand(new cageStop(this));
    setDefaultCommand(new cageReset(this));
  }

  @Override
  public void periodic() {
  }

  //Commands
  public void cageLift(){
    Motors.m_lift.set(-1);
  }
  public void cageStop(){
    Motors.m_lift.stopMotor();
  }
  public void cageDelift(){
    Motors.m_lift.set(1);
  }
  public void cageRelease(){
      Motors.m_release.setAngle(0);
  }
  public void cageReset(){
    Motors.m_release.setAngle(110);
}

}
