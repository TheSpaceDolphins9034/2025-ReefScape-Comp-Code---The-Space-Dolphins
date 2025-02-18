// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

//constants
import frc.robot.Constants.OIConstants;
//commands
import frc.robot.commands.Extra.resetGyroValue;
import frc.robot.commands.LimeLightFunctions.autoTrack;
import frc.robot.commands.ManualFunctions.CascadeLift.cascadeDown;
import frc.robot.commands.ManualFunctions.CascadeLift.cascadeStop;
import frc.robot.commands.ManualFunctions.CascadeLift.cascadeUp;
import frc.robot.commands.SetPositions.Coral.coralFeed;
//subsystems
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Drive.LimeLight;
import frc.robot.subsystems.Functions.Coral;
import frc.robot.subsystems.Functions.Cascade;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LimeLight m_limelight = new LimeLight();
  //private final Coral m_coral = new Coral();
  private final Cascade m_cascade = new Cascade();
  private final LoggedDashboardChooser<Command> autoChooser;
  // The driver's controller
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final Joystick m_operatorController = new Joystick(OIConstants.kOperatorControllerPort);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureJoystickButtonBindings();
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(zAxis(m_driverController.getRawAxis(2)), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
    autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
  }

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

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureJoystickButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
  }

  private void configureButtonBindings() {
    //          -- Establishing joystick Buttons --
    //Driver
    new JoystickButton(m_driverController, 11).whileTrue(new resetGyroValue(m_robotDrive));
    new JoystickButton(m_driverController, 1).onTrue(new autoTrack(m_limelight));
    //Operation Pad
    new JoystickButton(m_operatorController, 4).onTrue(new cascadeUp(m_cascade));
    new JoystickButton(m_operatorController, 3).onTrue(new cascadeStop(m_cascade));
    new JoystickButton(m_operatorController, 2).onTrue(new cascadeDown(m_cascade));
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_robotDrive.getAutonomousCommand("Foward + Rotate");
  }
}
