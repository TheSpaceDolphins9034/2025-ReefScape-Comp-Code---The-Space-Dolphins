// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Functions;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

//commands
import frc.robot.commands.ManualFunctions.CoralEffector.coralIntake;
import frc.robot.commands.ManualFunctions.CoralEffector.coralOuttake;
import frc.robot.commands.ManualFunctions.CoralEffector.coralStop;
import frc.robot.commands.SetPositions.Coral.coralFeed;

public class Coral extends SubsystemBase {
  /** Creates a new Coral. */
   private final AnalogInput m_ultraSonic = new AnalogInput(3);
  private final SparkMax m_coral;
  private final SparkMax m_cascade;
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  public Coral() {
    m_coral = new SparkMax(Constants.CoralID, MotorType.kBrushless);
    m_cascade = new SparkMax(Constants.CascadeID, MotorType.kBrushless);
    m_led = new AddressableLED(6);
    m_ledBuffer = new AddressableLEDBuffer(10);
    m_led.setLength(m_ledBuffer.getLength());

    //m_armMotorR = new CANSparkMax(Constants.armMotorR,MotorType.kBrushless);
    setDefaultCommand(new coralStop(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Distance", m_ultraSonic.getValue());
  }

  //manuals
  public void coralIntake(){
    m_coral.set(.2);
  }
  public void coralOuttake(){
    coralIntake();
  }
  public void coralStop(){
    m_coral.stopMotor();
  }

  //set positions
  public void coralFeed(){
    coralIntake();
  }
  public void coralL1(){
    //coralIntake.until(m_ultraSonic.getValue()<6);
  }
  public void coralL2(){
    coralIntake();
  }
  public void coralL3(){
    coralIntake();
  }
  public void coralL4(){
    coralIntake();
  }

}
