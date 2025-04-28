package frc.robot.commands.LimeLightFunctions;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ChaseTagCommand extends Command {

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

  private static final int TAG_TO_CHASE = 2;
  private static final Transform3d TAG_TO_GOAL = 
    new Transform3d(
        new Translation3d(1.5, 0.0, 0.0), 
        new Rotation3d(0.0, 0.0, Math.PI));

  private final DriveSubsystem driveSubsystem;
  private final Supplier<Pose2d> poseProvider;

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

  private Pose3d lastTargetPose;

  public ChaseTagCommand(DriveSubsystem driveSubsystem, Supplier<Pose2d> poseProvider) {
    this.driveSubsystem = driveSubsystem;
    this.poseProvider = poseProvider;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    lastTargetPose = null;
    var robotPose = poseProvider.get();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose2d = poseProvider.get();
    var robotPose =
        new Pose3d(
          robotPose2d.getX(),
          robotPose2d.getY(),
          0.0,
          new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians())
        );

    var limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    var validTarget = limelightTable.getEntry("tv").getDouble(0) == 1.0;
    if (validTarget) {
      var targetId = (int) limelightTable.getEntry("tid").getDouble(-1);
      if (targetId == TAG_TO_CHASE) {
        var botPoseArray = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
        if (botPoseArray.length == 6) {
          lastTargetPose = new Pose3d(
              botPoseArray[0], botPoseArray[1], botPoseArray[2],
              new Rotation3d(botPoseArray[3], botPoseArray[4], botPoseArray[5]));

          // Transform the tag's pose to set our goal
          var goalPose = lastTargetPose.transformBy(TAG_TO_GOAL).toPose2d();

          // Drive
          xController.setGoal(goalPose.getX());
          yController.setGoal(goalPose.getY());
          omegaController.setGoal(goalPose.getRotation().getRadians());
        }
      }
    }

    if (lastTargetPose == null) {
      // No target has been visible
      driveSubsystem.drive(0, 0, 0, false);
    } else {
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }

      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
      if (omegaController.atGoal()) {
        omegaSpeed = 0;
      }

      driveSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation())
      );
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
