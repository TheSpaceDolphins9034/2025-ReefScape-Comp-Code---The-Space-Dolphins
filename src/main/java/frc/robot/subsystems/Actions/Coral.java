// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Actions;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//Constants
import frc.robot.Constants.Motors;
//Commands
import frc.robot.commands.ManualFunctions.CoralEffector.coralStop;

public class Coral extends SubsystemBase {
  private final AnalogInput m_ultraSonic;
  public int distanceUS = 0;
  public Coral() { 
    m_ultraSonic = new AnalogInput(3);
    setDefaultCommand(new coralStop(this));
  }

  @Override
  public void periodic() {
    distanceUS = m_ultraSonic.getValue();
    SmartDashboard.putNumber("Distance", distanceUS);
  }

  //manuals
  public void coralIntake(){
    Motors.m_coral.set(.2);
  }
  public void coralOuttake(){
    Motors.m_coral.set(.5);
  }
  public void coralStop(){
    Motors.m_coral.stopMotor();
  }

  //set positions
  public void coralFeed(){
    if(m_ultraSonic.getValue() > 310){
      Motors.m_coral.set(-.25);
    }else if(m_ultraSonic.getValue() <= 310){
      Motors.m_coral.stopMotor();
    }
  }
}
