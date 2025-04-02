// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Actions;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//Constants
import frc.robot.Constants.Motors;
//Commands
import frc.robot.commands.ManualFunctions.CoralEffector.coralStop;

public class Coral extends SubsystemBase {
  public Coral() { 
    setDefaultCommand(new coralStop(this));
  }

  @Override
  public void periodic() {
  }

  //manuals
  public void coralOuttake(){
    Motors.m_coral.set(-.5);
  }
  public void coralStop(){
    Motors.m_coral.stopMotor();
  }

  //set positions
  public void coralFeed(){
    
  }
}