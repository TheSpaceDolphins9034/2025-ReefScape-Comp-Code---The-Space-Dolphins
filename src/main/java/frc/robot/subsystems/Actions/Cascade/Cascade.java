// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Actions.Cascade;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.commands.ManualFunctions.CascadeLift.cascadeStop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cascade extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public TalonFX leadMotor;
  public TalonFX followerMotor;
  public double realPosition;
  public Cascade() {
    leadMotor = new TalonFX(9);
    followerMotor = new TalonFX(4);

    leadMotor.getConfigurator().apply(CascadeConfigs.Cascade.leadConfig);
    followerMotor.getConfigurator().apply(CascadeConfigs.Cascade.followerConfig);
    
    followerMotor.setControl(new Follower(leadMotor.getDeviceID(), false));
    setDefaultCommand(new cascadeStop(this));
  }

  public void cascadeUp(){
    leadMotor.set(.1);
  }
  public void cascadeDown(){
    leadMotor.set(-.1);
  }

  public void cascadeStop(){
    leadMotor.stopMotor();
  }

  public void setPosition(double position){
    leadMotor.setControl(new PositionVoltage(position).withSlot(0));
  }
  
  public void zeroPosition(){
    leadMotor.setControl(new VoltageOut(0).withLimitReverseMotion(true).withOutput(0));
  }

  @Override
  public void periodic() {
    realPosition = leadMotor.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("CascasePos", realPosition);
  }
}
