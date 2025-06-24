// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;
//package frc.robot.subsystems.swervedrive;
//import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import com.pathplanner.lib.config.RobotConfig;
import frc.robot.Constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InitSubs;
import frc.utils.LimeLightHelpers.PoseEstimate;
import swervelib.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveSubsystem extends SubsystemBase {

  SwerveDrive swerveDrive;
  private final SwerveDriveKinematics kinematics;
  private final SimSwerveModule[] modules;

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private Pigeon2 m_seagullGyro = new Pigeon2(2); 
  private PigeonIMU m_seagullIMU = new PigeonIMU(2);
  public final PIDConstants pidTranslation = new PIDConstants(7, 0.0, .08);
  public final PIDConstants pidRotation = new PIDConstants(4, 0.0, .03);
  
  //private SwerveDriveOdometry swerveOdometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getYaw(), );

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_seagullIMU.getYaw()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
   setupPathPlanner(); //Call your AutoBuilder configuration here.
   // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    modules = new SimSwerveModule[]{
      new SimSwerveModule(),
      new SimSwerveModule(),
      new SimSwerveModule(),
      new SimSwerveModule()
    };
    kinematics = new SwerveDriveKinematics(
      Constants.Swerve.flModuleOffset, 
      Constants.Swerve.frModuleOffset, 
      Constants.Swerve.blModuleOffset, 
      Constants.Swerve.brModuleOffset
    );
  }

    private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_seagullIMU.getYaw()),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      },
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Math.PI / 36), // State measurement standard deviations
      VecBuilder.fill(0.5, 0.5, Math.PI / 6));  // Vision measurement standard deviations

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    SmartDashboard.putNumber("GyroValue", getHeading());
    m_poseEstimator.update(Rotation2d.fromDegrees(m_seagullIMU.getYaw()), 
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
  });
    m_odometry.update(
        Rotation2d.fromDegrees(m_seagullIMU.getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    zeroHeading();
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_seagullIMU.getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, 
              Rotation2d.fromDegrees(m_seagullIMU.getYaw()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void drive(ChassisSpeeds chassisSpeeds){
    drive(chassisSpeeds, true);
  }
  /* 
   *    AUTONOMOUS COMMANDS
   * 
   *   .26 SPEED = 4 FEET/S
   * 
   *    FROM DRIVERSTATION VIEW
   *    DriveXFeet(Feet, (+ = Foward)(- = Back));
   *    DriveYFeet(Feet, (+ = Right)(- = Left));
   * 
   */
  public void DriveXFeet(double xFeet, double direction){
    InitSubs.i_robotDrive.drive(.26*direction, 0, 0, false);
    Timer.delay(.25*xFeet);
    InitSubs.i_robotDrive.drive(0, 0, 0,false);
  }
  public void DriveYFeet(double yFeet, double direction){
    InitSubs.i_robotDrive.drive(0, -.26*direction, 0,false);
    Timer.delay(.25*yFeet);
    InitSubs.i_robotDrive.drive(0, 0, 0, false);
  }
  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
   // m_seagullGyro.reset(); corrales rant
    m_seagullIMU.setYaw(180);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_seagullIMU.getYaw()).getDegrees();
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

 
  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.Swerve.maxModuleSpeed);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setTargetState(targetStates[i]);
    }
  }

  class SimSwerveModule {
    private SwerveModulePosition currentPosition = new SwerveModulePosition();
    private SwerveModuleState currentState = new SwerveModuleState();

    public SwerveModulePosition getPosition() {
      return currentPosition;
    }

    public SwerveModuleState getState() {
      return currentState;
    }

    public void setTargetState(SwerveModuleState targetState) {
      currentState = SwerveModuleState.optimize(targetState, currentState.angle);

      currentPosition = new SwerveModulePosition(currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
    }
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState());
  }

  private void driveRobotRelative(ChassisSpeeds speeds){
    drive(speeds, false);
  }

private void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative)
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
    speeds = ChassisSpeeds.discretize(speeds, .01); //LoggedRobot.defaultPeriodSecs
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
}
  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_seagullGyro.getAngularVelocityZWorld().getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }


   public void setupPathPlanner()
  {

    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings(); //needs to be in a try/catch
      
      // Configure AutoBuilder last. Other sources say you can put this in your DriveSubsystem but I had errors doing that.
      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getRobotRelativeSpeeds, 
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    pidTranslation, // Translation PID constants
                    pidRotation // Rotation PID constants 
            ),
          config,
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Blue;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );
    } catch (Exception e)
    {
      e.printStackTrace();
    }
    //Uncomment this if you want to monitor the configuration confirmation.
    System.out.println(AutoBuilder.isConfigured());
  }
/* */
  public Command getAutonomousCommand(String pathName) {
    	return new PathPlannerAuto(pathName);
  }
 
  public Command driveToPose(Pose2d pose)
  {
	PathConstraints constraints = new PathConstraints(
        2.8, 2.8,  //these two are different instances of max speeds, you can make them the same. 
        2 * Math.PI, 2 * Math.PI);	//same for these two angular speeds

	// Since AutoBuilder is configured, we can use it to build pathfinding commands
    	return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
      );
  }
}