// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimeLightFunctions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.InitSubs;
import frc.robot.subsystems.Vision.LightHouse;
import frc.utils.LimeLightHelpers;

/* You should consider using the more terse Co

mmand factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class autoTrack extends Command {
  public autoTrack(LightHouse param_limeLight) {
    InitSubs.i_lightHouse = param_limeLight;
    addRequirements(InitSubs.i_lightHouse);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(LimeLightHelpers.getTV("limelight-kepler")){
      InitSubs.i_lightHouse.autoTrack(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
