// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Actions;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//Constants
import frc.robot.Constants.Motors;
//Commands
import frc.robot.commands.ManualFunctions.AlgaeEffector.algaeStop;

public class Algae extends SubsystemBase {
  public Algae() {
    setDefaultCommand(new algaeStop(this));
  }

  @Override
  public void periodic() {
  }

  //manuals
  public void algaeIntake(){
    Motors.m_algae.set(.4);
  }
  public void algaeOuttake(){
    Motors.m_algae.set(-.4);
  }
  public void algaeStop(){
    Motors.m_algae.stopMotor();
  }

}
