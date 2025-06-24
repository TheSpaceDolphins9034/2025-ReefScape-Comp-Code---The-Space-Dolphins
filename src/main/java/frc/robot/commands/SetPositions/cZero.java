// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SetPositions;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.controls.MusicTone;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint.MinMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.InitSubs;
import frc.robot.Constants.Motors;
import frc.robot.Constants.Positions;
import frc.robot.subsystems.Actions.Cascade.Cascade;
import frc.robot.subsystems.Actions.Wrist.Wrist;
import pabeles.concurrency.IntOperatorTask.Max;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class cZero extends Command {
  public boolean zeroPositionCheck;
 public cZero(Cascade param_cascade, Wrist param_wrist) {
    addRequirements(param_cascade);
    addRequirements(param_wrist);
  }
  
  @Override
  public void initialize() {
    InitSubs.i_cascade.cPositions(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(InitSubs.i_cascade.m_cEncoder.getPosition()<=5.0){
      InitSubs.i_wrist.wristPID.setReference(4, SparkBase.ControlType.kPosition);
    }else{
      InitSubs.i_wrist.wristPID.setReference(Positions.wLevels, SparkBase.ControlType.kPosition);
    }

    if ((InitSubs.i_cascade.m_cEncoder.getPosition()>=0-.1 && InitSubs.i_cascade.m_cEncoder.getPosition()<=0+.1) && (InitSubs.i_wrist.m_wEncoder.getPosition()>=4-.1 && InitSubs.i_wrist.m_wEncoder.getPosition()<=4+.1)) {
      zeroPositionCheck = true;
    }else{
      zeroPositionCheck = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return zeroPositionCheck;
  }
}
