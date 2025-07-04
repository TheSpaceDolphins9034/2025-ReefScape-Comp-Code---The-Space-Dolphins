// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualFunctions.CageLift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.InitSubs;
import frc.robot.Constants.Motors;
import frc.robot.subsystems.Actions.CageLift.Lift;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class cageDelift extends Command {
  public cageDelift(Lift param_lift) {
    // Use addRequirements() here to declare subsystem dependencies.
    InitSubs.i_lift = param_lift;
    addRequirements(InitSubs.i_lift); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    InitSubs.i_lift.cageDelift();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Motors.m_lift.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
