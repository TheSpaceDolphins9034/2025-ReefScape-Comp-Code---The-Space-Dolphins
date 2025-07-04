// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualFunctions.CoralEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.InitSubs;
import frc.robot.subsystems.Actions.GamePieceManipulators.Coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class coralOuttake extends Command {
  public coralOuttake(Coral param_coral) {
    InitSubs.i_coral = param_coral;
    addRequirements(InitSubs.i_coral); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    InitSubs.i_coral.coralOuttake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
