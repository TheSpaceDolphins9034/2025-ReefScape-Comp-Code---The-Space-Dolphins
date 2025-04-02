// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Actions;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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
    wristPIDValues = new PIDController(.125  , 0, 0);
    m_wEncoder = Motors.m_wrist.getEncoder();
    wMotorConfig = new SparkMaxConfig();

    wMotorConfig
      .idleMode(IdleMode.kBrake);

    wMotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

    wMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //position PIDS
      .p(.125)
      .d(0)
      .outputRange(-.5, .5);
    Motors.m_wrist.configure(wMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  
    setDefaultCommand(new wristStop(this));
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Postions", m_wEncoder.getPosition());
    setDefaultCommand(new wristStop(this));

  }

  //manuals
    public void wristDown(){
      Motors.m_wrist.set(-.35);
    }
    public void wristStill(){
      Motors.m_wrist.stopMotor();
    }
    public void wristStop(){
      Motors.m_wrist.stopMotor();
    }
    public void wristUp(){
      Motors.m_wrist.set(.35);
    }
  //Set positions
    /* 
    /algae
    */
      public void wAlgaeBarge(){
        wristPID.setReference(0, SparkBase.ControlType.kPosition);
      }
      public void wAlgaeFloorIntake(){
        wristPID.setReference(-22, SparkBase.ControlType.kPosition);
      }
      public void wAlgaeHolder(){
        wristPID.setReference(-3, SparkBase.ControlType.kPosition);
      }
      public void wAlgaeLevelGrab(){
        wristPID.setReference(0, SparkBase.ControlType.kPosition);
      }
      public void wAlgaeProcessor(){
        wristPID.setReference(0, SparkBase.ControlType.kPosition);
      }
    /* 
    /Coral
    */
      public void wCoralFeed(){
        wristPID.setReference(0, SparkBase.ControlType.kPosition);
      }
      public void wLevels(){
        wristPID.setReference(0, SparkBase.ControlType.kPosition);
      }
}
