// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SetPositions;

import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.InitSubs;
import frc.robot.subsystems.Actions.Wrist.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class wPosition extends Command {
  /** Creates a new wPosition. */
  public double wPositionValue;
  public boolean wPositionCheck;
 public wPosition(Wrist param_wrist, double position) {
    this.wPositionValue = position;
    InitSubs.i_wrist.wristPIDValues.setTolerance(0, 0.05);
    addRequirements(param_wrist);
    SmartDashboard.putBoolean("wristTF", false);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    InitSubs.i_wrist.wristPID.setReference(wPositionValue, SparkBase.ControlType.kPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (InitSubs.i_wrist.m_wEncoder.getPosition()>=wPositionValue-.1 && InitSubs.i_wrist.m_wEncoder.getPosition()<=wPositionValue+.1) {
      wPositionCheck = true;
    }else{
      wPositionCheck = false;
    }
    SmartDashboard.putBoolean("wristTF", wPositionCheck);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wPositionCheck;
  }
}
