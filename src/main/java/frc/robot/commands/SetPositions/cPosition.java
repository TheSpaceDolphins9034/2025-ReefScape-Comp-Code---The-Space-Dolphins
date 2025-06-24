// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SetPositions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.InitSubs;
import frc.robot.subsystems.Actions.Cascade.Cascade;

public class cPosition extends Command {
  public double cPositionValue;
  public boolean cPositionCheck;

 public cPosition(Cascade param_cascade, double position) {
    this.cPositionValue = position;
    addRequirements(param_cascade);
    SmartDashboard.putBoolean("cascadeTF", false);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    InitSubs.i_cascade.cPositions(cPositionValue);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (InitSubs.i_cascade.m_cEncoder.getPosition()>=cPositionValue-.1 && InitSubs.i_cascade.m_cEncoder.getPosition()<=cPositionValue+.1) {
      cPositionCheck = true;
    }else{
      cPositionCheck = false;
    }
    SmartDashboard.putBoolean("cascadeTF", cPositionCheck);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cPositionCheck;
  }
  
}
