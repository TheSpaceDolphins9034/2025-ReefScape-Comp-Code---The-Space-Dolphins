// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.InitSubs;
import frc.robot.Constants.OIConstants;
import frc.utils.LimeLightHelpers;

public class LightHouse extends SubsystemBase {
  /** Creates a new LimeLight(LightHouse). */
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public LightHouse() {}

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
    double targetingAngularVelocity = (LimeLightHelpers.getTX("LightHouse") * kP);

    // convert to radians per second for our drive method
    targetingAngularVelocity *= AutoConstants.kMaxAngularSpeedRadiansPerSecond;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with LightHouse's "ty" value
  // this works best if your LightHouse's mount height and target mount height are different.
  // if your LightHouse and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double lightHouse_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimeLightHelpers.getTY("LightHouse") * kP;
    targetingForwardSpeed *= AutoConstants.kMaxSpeedMetersPerSecond;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  double lightHouse_domain_proportional()
  {    
    double kP = .1;
    double targetingSidewaysSpeed = LimeLightHelpers.getTA("LightHouse") * kP;
    targetingSidewaysSpeed *= AutoConstants.kMaxSpeedMetersPerSecond;
    targetingSidewaysSpeed *= -1.0;
    return targetingSidewaysSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  //It completes the function all the way through
  public void autoTrack(boolean fieldRelative) {
     var rot_lightHouse = lightHouse_aim_proportional();
     double rot = 0.0;
      if(LimeLightHelpers.getTV("LightHouse")==true){
         rot = rot_lightHouse;
         if(rot_lightHouse >=-.5 || rot_lightHouse <=.5){
            SmartDashboard.putBoolean("AprilTagTrack", true);
         }
      }else{
         rot = -MathUtil.applyDeadband(zAxis(m_driverController.getRawAxis(2)), OIConstants.kDriveDeadband);
         SmartDashboard.putBoolean("AprilTagTrack", false);
      }
        //final var forward_lightHouse = lightHouse_range_proportional();
        //xSpeed = forward_lightHouse
        //while using LightHouse, turn off field-relative driving.
    
       // new RunCommand(() -> 
       
       InitSubs.i_robotDrive.drive(
              -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              rot,
              fieldRelative);
              //,
          //RobotContainer.m_robotDrive);
}
public void autoAlign(boolean fieldRelative) {
  var rot_lightHouse = lightHouse_aim_proportional();
  var forward_lightHouse = lightHouse_range_proportional();
  var sideways_lightHouse = lightHouse_domain_proportional();
  double rot = 0.0;
  double xSpeed = 0.0;
  double ySpeed = 0.0;
   if(LimeLightHelpers.getTV("lightHouse") == true){
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
}
}
