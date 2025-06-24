// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.InitSubs;
import frc.robot.Constants.OIConstants;
import frc.utils.LimeLightHelpers;

public class LightHouse extends SubsystemBase {
  /** Creates a new LimeLight(LightHouse). */
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public int[] allianceReefArray;
  public int[] allianceFeederArray;
  Optional<Alliance> alliance = DriverStation.getAlliance();

  private final PIDController distancePID;
  private final PIDController anglePID;

  private double tx;
  private double ta;
  private double ty;

  private double angleToTarget = 0;
  private double distanceError = 0;
  private double speed = 0;
  private double xSpeed = 0;
  private double ySpeed = 0;
  private double angleError = 0;
  private double rotationSpeed = 0;

  public LightHouse() {
    distancePID = new PIDController(1.0, 0, 0);
    anglePID = new PIDController(3.0, 0, 0);
    if(alliance.get() == Alliance.Red){
      allianceReefArray = new int[] {6,7,8,9,10,11};
      allianceFeederArray = new int[] {1,2};
     }else if (alliance.get() == Alliance.Blue){
      allianceReefArray = new int[] {17,18,19,20,21,22};
      allianceFeederArray = new int[] {12,13};
     }else{
      allianceReefArray = new int[] {6,7,8,9,10,11,17,18,19,20,21,22};
      allianceFeederArray = new int[] {1,2,12,13};
     }
  }
/* 
  public boolean isTrackingValidTag(){
    boolean hasTarget = LimeLightHelpers.getTV("LightHouse");
    //no tag in sight
    if (!hasTarget){
      return false;
    }
    
    double tagIdDouble = LimeLightHelpers.getFiducialID("LightHouse");
  }
*/
  private double zAxis(double rawZ){
    if(rawZ >= -.2 && rawZ <= .2){
      rawZ = 0;
    }
    else if(rawZ < -.2){
      rawZ = rawZ + .2;
    }
    else{
      rawZ = rawZ-.2;
    }
    rawZ = rawZ * 1.25;
    return rawZ;
  }

  public double lightHouse_aim_proportional()
  { 
    double kP = .008;
    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your LightHouse 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimeLightHelpers.getTX("limelight-kepler") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity = AutoConstants.kMaxAngularSpeedRadiansPerSecond*targetingAngularVelocity;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity = -targetingAngularVelocity;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with LightHouse's "ty" value
  // this works best if your LightHouse's mount height and target mount height are different.
  // if your LightHouse and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double lightHouse_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimeLightHelpers.getTY("limelight-kepler") * kP;
    targetingForwardSpeed *= AutoConstants.kMaxSpeedMetersPerSecond;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  double lightHouse_domain_proportional()
  {    
    double kP = .05;
    double targetingSidewaysSpeed = LimeLightHelpers.getTA("limelight-kepler") * kP;
    targetingSidewaysSpeed *= AutoConstants.kMaxSpeedMetersPerSecond;
    targetingSidewaysSpeed *= -.25;
    return targetingSidewaysSpeed;
  }

  @Override
  public void periodic() {
    tx = LimeLightHelpers.getTX("limelight-kepler");
    ty = LimeLightHelpers.getTY("limelight-kepler");
    ta = LimeLightHelpers.getTA("limelight-kepler");
  }

  
  //It completes the function all the way through
  public void autoTrack(boolean fieldRelative) {
     final var rot_lightHouse = lightHouse_aim_proportional();
     double rot = 0.0;
      if(LimeLightHelpers.getTV("limelight-kepler")==true /*&& LimeLightHelpers.getFiducialID("lightHouse") == allianceReefArray.*/){
         rot = rot_lightHouse;
      }else{
         rot = -MathUtil.applyDeadband(zAxis(m_driverController.getRawAxis(2)), OIConstants.kDriveDeadband);
      }
        //final var forward_lightHouse = lightHouse_range_proportional();
        //xSpeed = forward_lightHouse
        //while using LightHouse, turn off field-relative driving.
    
       InitSubs.i_robotDrive.drive(
              -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              rot,
              fieldRelative);
}
public void autoAlign(boolean fieldRelative) {
  
  //gets current pose of the robot
  Pose2d currentPose = InitSubs.i_robotDrive.getPose();
  
  //detects if theres even a april tag
  if(ta == 0.0){
    InitSubs.i_robotDrive.drive(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(zAxis(m_driverController.getRawAxis(2)), OIConstants.kDriveDeadband),
        true);
    return;
  }
  

  //error between current and target pose
  angleToTarget = Math.atan2(ty, tx);
  distanceError = Math.sqrt(tx*tx+ty*ty);
  //PID for translation
  speed = distancePID.calculate(distanceError, 0);
  //use sin/cos to convert the angle to x and y speeds
  xSpeed = speed * Math.cos(angleToTarget);
  ySpeed = speed * Math.sin(angleToTarget);
  //PID Control for rotation
  angleError = angleToTarget - currentPose.getRotation().getRadians();
  rotationSpeed = anglePID.calculate(angleError, 0);
  //drive the robot to align with the april tag
  InitSubs.i_robotDrive.drive(xSpeed, ySpeed, rotationSpeed, fieldRelative);

/* 
   if(LimeLightHelpers.getTV("lightHouse") == true /*&& LimeLightHelpers.getFiducialID("lightHouse") : allianceReefArray){
      rot = rot_lightHouse;
      xSpeed = forward_lightHouse;
      ySpeed = sideways_lightHouse;
      SmartDashboard.putBoolean("AprilTagTrack", true);
   }else{
      xSpeed = -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), OIConstants.kDriveDeadband);
      ySpeed = -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), OIConstants.kDriveDeadband);
      rot = -MathUtil.applyDeadband(zAxis(m_driverController.getRawAxis(2)), OIConstants.kDriveDeadband);
      SmartDashboard.putBoolean("AprilTagTrack", false);
   }
     //while using lighHouse, turn off field-relative driving.
    InitSubs.i_robotDrive.drive(xSpeed, ySpeed, rot, fieldRelative);
    */
}
}
