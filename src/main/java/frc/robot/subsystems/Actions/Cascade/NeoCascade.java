/* 
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Actions.Cascade;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//Constants
import frc.robot.Constants.Motors;
//Commands
import frc.robot.commands.ManualFunctions.CascadeLift.cascadeStop;

public class NeoCascade extends SubsystemBase {
  public RelativeEncoder m_cEncoder;
  public RelativeEncoder m_c2Encoder;
  public SparkClosedLoopController cascadePID;
  public SparkClosedLoopController cascadePID2;
  public SparkMaxConfig cMotorConfig;
  public SparkMaxConfig c2MotorConfig;
  public PIDController cascadePIDValues;

  public double speed;
  public NeoCascade() {
    cascadePID = Motors.m_cascade.getClosedLoopController();
    cascadePID2 = Motors.m_cascade2.getClosedLoopController();
    m_cEncoder = Motors.m_cascade.getEncoder();
    m_c2Encoder = Motors.m_cascade2.getEncoder();
    cMotorConfig = new SparkMaxConfig();
    c2MotorConfig = new SparkMaxConfig();

    cascadePIDValues = new PIDController(.1  , 0, .1);
 
    //Cascade Motor 1
    cMotorConfig
      .idleMode(IdleMode.kBrake);

    cMotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

    cMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //position PIDS
      .p(.1)
      .d(.1)
      .outputRange(-1, .75);
    Motors.m_cascade.configure(cMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    //Cascade Motor 2
    c2MotorConfig.follow(Motors.m_cascade);

    c2MotorConfig.idleMode(IdleMode.kBrake);
   
    c2MotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

    c2MotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //position PIDS
      .p(.1)
      .d(.1)
      .outputRange(-1, .75);

    Motors.m_cascade2.configure(c2MotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    setDefaultCommand(new cascadeStop(this));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Cascade Encoder", -m_cEncoder.getPosition());
  }

  public boolean cIsAtPosition(double position){
    return Math.abs(m_cEncoder.getPosition() - position) < 0.1;
  }

  //-180 is the limit of the cascade

  //manuals
  public void cascadeUp(){
    Motors.m_cascade.set(-1);
    Motors.m_cascade2.set(-1);
  }
  public void cascadeStop(){
    Motors.m_cascade.stopMotor();
    Motors.m_cascade2.stopMotor();
  }
  public void cascadeDown(){
      Motors.m_cascade.set(.75);
      Motors.m_cascade2.set(.75);
  }
  public void cascadeBelow0(){
    cascadePID.setReference(0, SparkBase.ControlType.kPosition);
  }

  //Set positions
    /* 
    /algae
    //
      public void cAlgaeBarge(){
        cascadePID.setReference(0, SparkBase.ControlType.kPosition);
      }
      public void cAlgaeFloorIntake(){
        cascadePID.setReference(0, SparkBase.ControlType.kPosition);
      }
      public void cAlgaeHolder(){
        cascadePID.setReference(-25, SparkBase.ControlType.kPosition);
      }
      public void cAlgaeL1(){
        cascadePID.setReference(-50, SparkBase.ControlType.kPosition);
      }
      public void cAlgaeL2(){
        cascadePID.setReference(0, SparkBase.ControlType.kPosition);
      }
      public void cAlgaeProcessor(){
        cascadePID.setReference(0, SparkBase.ControlType.kPosition);
      }
    /* 
    /Coral
    //
      public void cCoralFeed(){
        cascadePID.setReference(0, SparkBase.ControlType.kPosition);
      }
      public void cCoralL1(){
        cascadePID.setReference(-45, SparkBase.ControlType.kPosition);
      }
      public void cCoralL2(){
        cascadePID.setReference(-75, SparkBase.ControlType.kPosition);
      }
      public void cCoralL3(){
        cascadePID.setReference(-105, SparkBase.ControlType.kPosition);
      }
      public void cCoralL4(){
        cascadePID.setReference(-155.5, SparkBase.ControlType.kPosition);
        //+11 after sequence
      }
}
*/
