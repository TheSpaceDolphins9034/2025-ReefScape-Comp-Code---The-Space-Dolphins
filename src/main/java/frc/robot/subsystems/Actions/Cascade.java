// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Actions;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//Constants
import frc.robot.Constants.Motors;
//Commands
import frc.robot.commands.ManualFunctions.CascadeLift.cascadeStop;

public class Cascade extends SubsystemBase {
  public RelativeEncoder m_cEncoder;
  public SparkClosedLoopController cascadePID;
  public SparkMaxConfig cMotorConfig;
  public PIDController cascadePIDValues;
  public Cascade() {
   cascadePID = Motors.m_cascade.getClosedLoopController(); 
    cascadePIDValues = new PIDController(.35, 0, 0);
    m_cEncoder = Motors.m_cascade.getEncoder();
    cMotorConfig = new SparkMaxConfig();

    cMotorConfig
      .idleMode(IdleMode.kBrake);

    cMotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

    cMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //position PIDS
      .p(5)
      .d(0)
      .outputRange(-1, 1)
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(0, 1);
    Motors.m_cascade.configure(cMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Cascade Encoder", m_cEncoder.getPosition());
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
        /*
        if(m_cascadeEncoder.get()>300){
          Motors.m_coral.set(-.25);
        }
          */
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
        Motors.m_wrist.set(MathUtil.clamp(cascadePIDValues.calculate(m_cEncoder.getPosition(), -24), -1, 1));
      }
}
