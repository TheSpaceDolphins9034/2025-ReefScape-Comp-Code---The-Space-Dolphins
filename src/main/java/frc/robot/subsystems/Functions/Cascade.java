// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Functions;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//commands
import frc.robot.commands.ManualFunctions.CascadeLift.cascadeStop;

public class Cascade extends SubsystemBase {
  /** Creates a new Cascade. */
  private final SparkMax m_cascade;
  double speed = 0;
  public Cascade() {
    m_cascade = new SparkMax(Constants.CascadeID, MotorType.kBrushless);
    setDefaultCommand(new cascadeStop(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("speed", speed);
  }

  //manuals
  public void cascadeUp(){
    m_cascade.set(1);
  }
  public void cascadeStop(){
    m_cascade.stopMotor();
  }
  public void cascadeDown(){
    m_cascade.set(-.5);
  }
}
