// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SetPositions;

import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.InitSubs;
import frc.robot.subsystems.Actions.Cascade;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class cPosition extends Command {
  /** Creates a new wPosition. */
  public double position;
 public cPosition(Cascade param_cascade, double position) {
    this.position = position;
    InitSubs.i_cascade.cascadePIDValues.setTolerance(0, 0.05);
    addRequirements(param_cascade);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    InitSubs.i_cascade.cascadePIDValues.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    InitSubs.i_cascade.cascadePID.setReference(position, SparkBase.ControlType.kPosition);
    if(isFinished()){
      end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return InitSubs.i_cascade.cascadePIDValues.atSetpoint();
  }
}
