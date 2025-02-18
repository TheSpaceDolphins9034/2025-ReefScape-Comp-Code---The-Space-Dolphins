// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualFunctions.CascadeLift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Functions.Cascade;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class cascadeUp extends Command {
  /** Creates a new resetGyroValue. */
  Cascade m_cascade;
  public cascadeUp(Cascade param_cascade) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cascade = param_cascade;
    addRequirements(m_cascade); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_cascade.cascadeUp();
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
