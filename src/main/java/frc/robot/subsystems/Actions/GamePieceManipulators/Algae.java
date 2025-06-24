// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Actions.GamePieceManipulators;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//Constants
import frc.robot.Constants.Motors;
//Commands
import frc.robot.commands.ManualFunctions.AlgaeEffector.algaeStop;

public class Algae extends SubsystemBase {
  public Algae() {
    Motors.m_algae.configure(ManipulatorConfigs.GameManipulators.algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    setDefaultCommand(new algaeStop(this));
    SmartDashboard.putBoolean("IntakeFoward", false);
  }

  @Override
  public void periodic(){
  }

  //manuals
  public void algaeIntake(){
    Motors.m_algae.set(-.75);
    SmartDashboard.putBoolean("IntakeFoward", true);
  }
  public void algaeOuttake(){
    Motors.m_algae.set(.75);
    SmartDashboard.putBoolean("IntakeFoward", false);
  }
  public void algaeStop(){
    Motors.m_algae.stopMotor();
    SmartDashboard.putBoolean("IntakeFoward", false);
  }

}
