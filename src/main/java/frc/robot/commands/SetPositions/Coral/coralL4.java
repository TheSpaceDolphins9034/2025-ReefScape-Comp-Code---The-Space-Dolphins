// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SetPositions.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Functions.Coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class coralL4 extends Command {
  /** Creates a new resetGyroValue. */
  Coral m_coral;
  public coralL4(Coral param_coral) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_coral = param_coral;
    addRequirements(m_coral); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
