// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimeLightFunctions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.utils.LimeLightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class autoAlign extends Command {
  private DriveSubsystem swerve;
  private boolean rightSide;
  private boolean foward;
  private boolean strafe;
  private boolean turn;

  private PIDController fowardController = new PIDController(2.5, 0, 0.001);
  private PIDController strafeController = new PIDController(2.5, 0, 0.001);
  private PIDController rotationController = new PIDController(.075, 0, 0.001);

  private static double bumperLength = .914;
  private static double bumperWidth = .914;
  private static double reefSeparation = .33;

  public autoAlign(DriveSubsystem swerve, boolean rightSide, boolean foward, boolean strafe, boolean turn) {
    this.swerve = swerve;
    this.rightSide = rightSide;
    this.foward = foward;
    this.strafe = strafe;
    this.turn = turn;

    fowardController.setTolerance(0.05, 0.05);
    strafeController.setTolerance(0.05, 0.05);
    rotationController.setTolerance(0.8, 2);

    addRequirements(swerve);
  }
  public autoAlign(DriveSubsystem swerve, boolean rightSide){
    this(swerve, rightSide, true, true, true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    fowardController.reset();
    strafeController.reset();
    rotationController.reset();
    LimeLightHelpers.setLEDMode_ForceOn("limelight-kepler");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose3d pose = LimeLightHelpers.getTargetPose3d_RobotSpace("limelight-kepler");
    if (pose.getTranslation().getNorm() != 0 && pose.getRotation().getAngle() != 0 && pose.getTranslation().getNorm() <1.5) {
      double rotationTarget = rotationController.calculate(Math.toDegrees(pose.getRotation().getY()),9);
      double translationTarget = strafeController.calculate(pose.getX(), .35 * (rightSide ? -reefSeparation : reefSeparation));
      double fowardTarget = -fowardController.calculate(pose.getZ(), bumperLength);
      swerve.drive(new ChassisSpeeds(
        fowardTarget, 
        translationTarget, 
        rotationTarget
        ));
    }
    SmartDashboard.putNumber("Vision Angle", Math.toDegrees(pose.getRotation().getX()));
    SmartDashboard.putNumber("Vision Offset", pose.getX());
    SmartDashboard.putNumber("Vision Distance", pose.getTranslation().getNorm());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotationController.atSetpoint() && strafeController.atSetpoint() && fowardController.atSetpoint();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds());
    LimeLightHelpers.setLEDMode_ForceOff("limelight-kepler");
  }
}