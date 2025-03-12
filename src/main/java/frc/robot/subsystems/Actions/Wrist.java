// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Actions;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.jni.REVLibJNI;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//Constants
import frc.robot.Constants.Motors;
//Commands
import frc.robot.commands.ManualFunctions.ArmWrist.wristStop;

public class Wrist extends SubsystemBase {
  public RelativeEncoder m_wEncoder;
  public SparkClosedLoopController wristPID;
  public SparkMaxConfig wMotorConfig;
  public PIDController wristPIDValues;
  //private RelativeEncoder m_wBoreEncoder;
  public Wrist() {
    //m_wBoreEncoder = Motors.m_wrist.getAlternateEncoder();
    wristPID = Motors.m_wrist.getClosedLoopController(); 
    wristPIDValues = new PIDController(.35  , 0, 0);
    m_wEncoder = Motors.m_wrist.getEncoder();
    wMotorConfig = new SparkMaxConfig();

    wMotorConfig
      .idleMode(IdleMode.kBrake);

    wMotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

    wMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //position PIDS
      .p(5)
      .d(0)
      .outputRange(-1, 1)
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(0, 1);
    Motors.m_wrist.configure(wMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    setDefaultCommand(new wristStop(this));
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Postions", m_wEncoder.getPosition());
  }

  //manuals
    public void wristDown(){
      Motors.m_wrist.set(-.1);
    }
    public void wristStop(){
      Motors.m_wrist.stopMotor();
    }
    public void wristUp(){
      Motors.m_wrist.set(.1);
    }
  //Set positions
    /* 
    /algae
    */
      public void wAlgaeBarge(){
        Motors.m_wrist.set(MathUtil.clamp(wristPIDValues.calculate(m_wEncoder.getPosition(), 0), -1, 1));
      }
      public void wAlgaeFloorIntake(){
        //wristPID.setReference(10, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        Motors.m_wrist.set(MathUtil.clamp(wristPIDValues.calculate(m_wEncoder.getPosition(), -5), -1, 1));
      }
      public void wAlgaeHolder(){
        Motors.m_wrist.set(MathUtil.clamp(wristPIDValues.calculate(m_wEncoder.getPosition(), -24), -1, 1));
      }
      public void wAlgaeLevelGrab(){
  
      }
      public void wAlgaeProcessor(){
  
      }
    /* 
    /Coral
    */
      public void wCoralFeed(){
        
      }
      public void wLevels(){
        Motors.m_wrist.set(MathUtil.clamp(wristPIDValues.calculate(m_wEncoder.getPosition(), 0), -1, 1));
      }
}
