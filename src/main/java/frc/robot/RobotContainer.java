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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.InitSubs;
//constants
import frc.robot.Constants.OIConstants;
//commands
import frc.robot.commands.Extra.resetGyroValue;
import frc.robot.commands.LimeLightFunctions.autoTrack;
import frc.robot.commands.ManualFunctions.AlgaeEffector.algaeIntake;
import frc.robot.commands.ManualFunctions.CageLift.cageDelift;
import frc.robot.commands.ManualFunctions.CageLift.cageLift;
import frc.robot.commands.SetPositions.Cascade.cCoralL4;
import frc.robot.commands.SetPositions.Wrist.wCoralL4;
import frc.robot.commands.LimeLightFunctions.autoAlign;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The driver's controller
  public static final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final Joystick m_operatorController = new Joystick(OIConstants.kOperatorControllerPort);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureJoystickButtonBindings();
    // Configure default commands
    InitSubs.i_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> InitSubs.i_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(zAxis(m_driverController.getRawAxis(2)), OIConstants.kDriveDeadband),
                true),
            InitSubs.i_robotDrive));
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
            () -> InitSubs.i_robotDrive.setX(),
            InitSubs.i_robotDrive));
  }

  private void configureButtonBindings() {

    //          -- Establishing joystick Buttons --
    //Driver
    new JoystickButton(m_driverController, 11).whileTrue(new resetGyroValue(InitSubs.i_robotDrive));
    new JoystickButton(m_driverController, 1).whileTrue(new autoTrack(InitSubs.i_lightHouse));
    new JoystickButton(m_driverController, 8).whileTrue(new autoAlign(InitSubs.i_lightHouse));

    //Operation Pad
    //ultra sonic coral intake
      //new JoystickButton(m_operatorController, 8).whileTrue(new coralFeed(m_coral));
      //new JoystickButton(m_operatorController, 8).onFalse(new coralStop(m_coral));
    //Cage Lift
      new JoystickButton(m_operatorController,10).whileTrue(new cageLift(InitSubs.i_lift));
      new JoystickButton(m_operatorController,9).whileTrue(new cageDelift(InitSubs.i_lift));
    //Algae Effector
      new JoystickButton(m_operatorController,8).whileTrue(new algaeIntake(InitSubs.i_algae));
      new JoystickButton(m_operatorController,7).whileTrue(new algaeIntake(InitSubs.i_algae));
    //Set Positions
      new JoystickButton(m_operatorController,111).onTrue(
        new SequentialCommandGroup(
          new cCoralL4(InitSubs.i_cascade),
          new wCoralL4(InitSubs.i_wrist)
        )
      );

    /* ECT.
    new JoystickButton(m_operatorController, 3).onTrue(new cascadeStop(m_cascade));
    new JoystickButton(m_operatorController, 4).whileTrue(new cascadeUp(m_cascade));
    new JoystickButton(m_operatorController, 3).onTrue(new cascadeStop(m_cascade));
    new JoystickButton(m_operatorController, 2).whileTrue(new cascadeDown(m_cascade));
    */
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return InitSubs.i_robotDrive.getAutonomousCommand("Foward + Rotate");
  }
}
