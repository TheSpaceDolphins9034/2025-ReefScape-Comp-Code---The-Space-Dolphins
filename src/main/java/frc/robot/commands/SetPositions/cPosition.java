// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SetPositions;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.controls.MusicTone;

import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint.MinMax;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Actions.Cascade.Cascade;
import pabeles.concurrency.IntOperatorTask.Max;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class cPosition extends Command {
  /** Creates a new wPosition. */
  public Orchestra m_orchestra = new Orchestra();
  public double position;
  public static Cascade i_cascade = new Cascade();
 public cPosition(Cascade param_cascade, double position) {
    m_orchestra.addInstrument(i_cascade.leadMotor);
    m_orchestra.loadMusic("track.chrp");
    this.position = position;
    addRequirements(param_cascade);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    i_cascade.setPosition(position);
    //i_cascade.m_Motor1.setControl(i_cascade.m_positionVoltage.withPosition(position));
    //i_cascade.m_Motor2.setControl(i_cascade.m_positionVoltage.withPosition(position));
    //i_cascade.m_Motor1.setControl(i_cascade.m_PositionDutyCycle.withPosition(position));
    //i_cascade.m_Motor2.setControl(i_cascade.m_PositionDutyCycle.withPosition(position));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_orchestra.play();
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
