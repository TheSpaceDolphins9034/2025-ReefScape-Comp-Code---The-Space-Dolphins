// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Actions.CageLift;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//Constants
import frc.robot.Constants.Motors;

public class Lift extends SubsystemBase {
  public boolean released = false;
  public Lift() {
  }

  @Override
  public void periodic() {
  }

  //Commands
  public void cageLift(){
    Motors.m_lift.set(-1);
  }
  public void cageDelift(){
    Motors.m_lift.set(1);
  }
}
