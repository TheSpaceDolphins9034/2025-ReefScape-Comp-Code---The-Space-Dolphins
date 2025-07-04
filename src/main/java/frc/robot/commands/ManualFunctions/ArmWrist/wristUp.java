// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualFunctions.ArmWrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.InitSubs;
import frc.robot.Constants.Motors;
import frc.robot.subsystems.Actions.Wrist.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class wristUp extends Command {
  public wristUp(Wrist param_wrist) {
    InitSubs.i_wrist = param_wrist;
    addRequirements(InitSubs.i_wrist); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    InitSubs.i_wrist.wristUp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Motors.m_wrist.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
