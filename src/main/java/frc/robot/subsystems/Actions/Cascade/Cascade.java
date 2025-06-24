
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Actions.Cascade;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InitSubs;
//Constants
import frc.robot.Constants.Motors;

public class Cascade extends SubsystemBase {

  public RelativeEncoder m_cEncoder = Motors.m_cLead.getEncoder();
  public RelativeEncoder m_c2Encoder = Motors.m_cFollower.getEncoder();
  public SparkClosedLoopController cascadePID = Motors.m_cLead.getClosedLoopController();
  public SparkClosedLoopController cascadePID2 = Motors.m_cFollower.getClosedLoopController();
  public PIDController cascadePIDValues = new PIDController(.1, 0, 1);
  public double speed;

  public Cascade() {
    cascadePIDValues.setTolerance(0.05, 0.05);
    cascadePIDValues.reset();
    Motors.m_cLead.configure(CascadeConfigs.Cascade.cLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    Motors.m_cFollower.configure(CascadeConfigs.Cascade.cFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Cascade Encoder", -m_cEncoder.getPosition());
  }
  
  //-180 is the limit of the cascade
  //manuals
  public void cascadeUp(){
    Motors.m_cLead.set(-1);
  }
  public void cascadeDown(){
    Motors.m_cLead.set(.75);
  }

  //Set positions
    public void cPositions(double position){
      InitSubs.i_cascade.cascadePID.setReference(position, SparkBase.ControlType.kPosition);
    }
}
