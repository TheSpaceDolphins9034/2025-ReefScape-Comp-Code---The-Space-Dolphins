// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.InitSubs;
import frc.robot.subsystems.Vision.LightHouse;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public final SendableChooser<String> m_chooser = new SendableChooser<>();
  private RobotContainer m_robotContainer;

  LightHouse m_limelight = new LightHouse();

  //AUTOS
  //default
  private static final String ANull = null;
  //real
  private static final String AAlgaeLineUpLeft = "AlgaeLineUpLeft";
  private static final String ALAlgaeLineUpMid = "LAlgaeLineUpMid";
  private static final String ARAlgaeLineUpMid = "RAlgaeLineUpMid";
  private static final String AAlgaeLineUpRight = "AlgaeLineUpRight";
  private static final String AL4Left = "L4Left";
  private static final String ALeave = "Leave";
  private static final String AL4Leave = "L4Leave";
  //tests
  private static final String ARotate = "Rotate";
  private static final String AFowardRotate = "Foward + Rotate";
  private static final String AFoward = "Foward";
  private static final String ASwivleRotate = "Swivle + Rotate";
  private static final String ASwivle = "Swivle";

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //default
    m_chooser.setDefaultOption("Nothing", ANull);
    //Real Autos
    m_chooser.addOption(ARAlgaeLineUpMid, ARAlgaeLineUpMid);
    m_chooser.addOption(ALAlgaeLineUpMid, ALAlgaeLineUpMid);
    m_chooser.addOption(AAlgaeLineUpRight, AAlgaeLineUpRight);
    m_chooser.addOption(AAlgaeLineUpLeft, AAlgaeLineUpLeft);

    m_chooser.addOption(AL4Left, AL4Left);
    m_chooser.addOption(ALeave, ALeave);
    m_chooser.addOption(AL4Leave, AL4Leave);
    //Test Autos
    m_chooser.addOption(ARotate, ARotate);
    m_chooser.addOption(AFoward, AFoward);
    m_chooser.addOption(ASwivle, ASwivle);
    m_chooser.addOption(ASwivleRotate, ASwivleRotate);
    m_chooser.addOption(AFowardRotate, AFowardRotate);
  
    SmartDashboard.putNumber("Match Time", 0);
    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putBoolean("DriverController", false);
    SmartDashboard.putBoolean("OperatorController", false);
  
    InitSubs.i_robotDrive.zeroHeading();

    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putBoolean("DriverController", DriverStation.isJoystickConnected(0));
    SmartDashboard.putBoolean("OperatorController", DriverStation.isJoystickConnected(1));
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    InitSubs.i_robotDrive.resetOdometry(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
    m_autonomousCommand = InitSubs.i_robotDrive.getAutonomousCommand(m_chooser.getSelected());
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }
    
      @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
 
}
