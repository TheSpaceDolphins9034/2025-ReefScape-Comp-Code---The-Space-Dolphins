// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LimeLightFunctions.autoTrack;
import frc.utils.LimeLightHelpers;

public class LimeLight extends SubsystemBase {
  /** Creates a new LimeLight. */
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public LimeLight() {}

  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = 1;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimeLightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= AutoConstants.kMaxAngularSpeedRadiansPerSecond;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimeLightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= AutoConstants.kMaxSpeedMetersPerSecond;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //It completes the function all the way through
  public void autoTrack(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed =
    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband);

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed =
    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband);

        final var rot_limelight = limelight_aim_proportional();
        var rot = rot_limelight;
        /* 
        final var forward_limelight = limelight_range_proportional();
        xSpeed = forward_limelight;
        */
        //while using Limelight, turn off field-relative driving.
    
      RobotContainer.m_robotDrive.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
}
