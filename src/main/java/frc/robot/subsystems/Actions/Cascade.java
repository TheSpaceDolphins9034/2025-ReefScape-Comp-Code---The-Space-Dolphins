// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Actions;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//Constants
import frc.robot.Constants.Motors;
//Commands
import frc.robot.commands.ManualFunctions.CascadeLift.cascadeStop;

public class Cascade extends SubsystemBase {
  public Cascade() {
    setDefaultCommand(new cascadeStop(this));
  }

  @Override
  public void periodic() {
  }

  //manuals
  public void cascadeUp(){
    Motors.m_cascade.set(1);
  }
  public void cascadeStop(){
    Motors.m_cascade.stopMotor();
  }
  public void cascadeDown(){
    Motors.m_cascade.set(-.5);
  }

  //Set positions
    /* 
    /algae
    */
      public void cAlgaeBarge(){
    
      }
      public void cAlgaeFloorIntake(){
    
      }
      public void cAlgaeHolder(){
    
      }
      public void cAlgaeL1(){
    
      }
      public void cAlgaeL2(){
    
      }
      public void cAlgaeProcessor(){
    
      }
    /* 
    /Coral
    */
      public void cCoralFeed(){
    
      }
      public void cCoralL1(){
    
      }
      public void cCoralL2(){
    
      }
      public void cCoralL3(){
    
      }
      public void cCoralL4(){
    
      }
}
