// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Actions;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//Constants
import frc.robot.Constants.Motors;
//Commands
import frc.robot.commands.ManualFunctions.ArmWrist.wristStop;

public class Wrist extends SubsystemBase {
  public Wrist() {
    setDefaultCommand(new wristStop(this));
  }

  @Override
  public void periodic() {
  }

  //manuals
    public void wristDown(){

    }
    public void wristStop(){
      Motors.m_wrist.stopMotor();
    }
    public void wristUp(){
      
    }
  //Set positions
    /* 
    /algae
    */
      public void wAlgaeBarge(){
    
      }
      public void wAlgaeFloorIntake(){
  
      }
      public void wAlgaeHolder(){
  
      }
      public void wAlgaeL1(){
  
      }
      public void wAlgaeL2(){
  
      }
      public void wAlgaeProcessor(){
  
      }
    /* 
    /Coral
    */
      public void wCoralFeed(){
  
      }
      public void wCoralL1(){
  
      }
      public void wCoralL2(){
  
      }
      public void wCoralL3(){
  
      }
      public void wCoralL4(){
  
      }
}
