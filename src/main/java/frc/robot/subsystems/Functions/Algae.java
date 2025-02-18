// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Functions;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

//commands
import frc.robot.commands.ManualFunctions.AlgaeEffector.algaeIntake;
import frc.robot.commands.ManualFunctions.AlgaeEffector.algaeOuttake;
import frc.robot.commands.ManualFunctions.AlgaeEffector.algaeStop;
import frc.robot.commands.SetPositions.Algae.algaeL1;
import frc.robot.commands.SetPositions.Algae.algaeL2;
import frc.robot.commands.SetPositions.Algae.algaeProcessor;
import frc.robot.commands.SetPositions.Algae.algaeFloorIntake;
import frc.robot.commands.SetPositions.Algae.algaeHolder;
import frc.robot.commands.SetPositions.Algae.algaeBarge;

public class Algae extends SubsystemBase {
  /** Creates a new Coral. */
  private final SparkMax m_algae;
  public Algae() {
    m_algae = new SparkMax(Constants.CoralID, MotorType.kBrushless);
    setDefaultCommand(new algaeStop(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //manuals
  public void algaeIntake(){
    m_algae.set(.3);
  }
  public void algaeOuttake(){
    m_algae.set(-.3);
  }
  public void algaeStop(){
    m_algae.stopMotor();
  }
  
  //set positions
  public void algaeL1(){
    
  }
  public void algaeL2(){
    
  }
  public void algaeFloorIntake(){
    
  }
  public void algaeHolder(){
    
  }
  public void algaeBarge(){
    
  }
  public void algaeProcessor(){
    
  }

}
