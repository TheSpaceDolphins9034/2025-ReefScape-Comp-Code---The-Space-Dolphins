// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//constants
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Positions;
import frc.robot.Constants.InitSubs;
//commands
import frc.robot.commands.Extra.resetGyroValue;
import frc.robot.commands.LimeLightFunctions.autoTrack;
import frc.robot.commands.ManualFunctions.AlgaeEffector.algaeIntake;
import frc.robot.commands.ManualFunctions.AlgaeEffector.algaeOuttake;
import frc.robot.commands.ManualFunctions.AlgaeEffector.algaeStop;
import frc.robot.commands.ManualFunctions.ArmWrist.wristDown;
import frc.robot.commands.ManualFunctions.ArmWrist.wristStop;
import frc.robot.commands.ManualFunctions.ArmWrist.wristUp;
import frc.robot.commands.ManualFunctions.CageLift.cageDelift;
import frc.robot.commands.ManualFunctions.CageLift.cageLift;
import frc.robot.commands.ManualFunctions.CageLift.cageRelease;
import frc.robot.commands.ManualFunctions.CageLift.cageStop;
import frc.robot.commands.ManualFunctions.CascadeLift.cascadeDown;
import frc.robot.commands.ManualFunctions.CascadeLift.cascadeStop;
import frc.robot.commands.ManualFunctions.CascadeLift.cascadeUp;
import frc.robot.commands.ManualFunctions.CoralEffector.coralOuttake;
import frc.robot.commands.ManualFunctions.CoralEffector.coralStop;
import frc.robot.commands.SetPositions.cPosition;
import frc.robot.commands.SetPositions.wPosition;
import frc.robot.commands.SetPositions.Cascade.cAlgaeBarge;
import frc.robot.commands.SetPositions.Cascade.cAlgaeFloorIntake;
import frc.robot.commands.SetPositions.Cascade.cAlgaeHolder;
import frc.robot.commands.SetPositions.Cascade.cAlgaeL1;
import frc.robot.commands.SetPositions.Cascade.cAlgaeL2;
import frc.robot.commands.SetPositions.Cascade.cAlgaeProcessor;
import frc.robot.commands.SetPositions.Cascade.cCoralFeed;
import frc.robot.commands.SetPositions.Wrist.wAlgaeLevelGrab;
import frc.robot.commands.SetPositions.Wrist.wAlgaeProcessor;
import frc.robot.commands.SetPositions.Wrist.wAlgaeBarge;
import frc.robot.commands.SetPositions.Wrist.wAlgaeFloorIntake;
import frc.robot.commands.SetPositions.Wrist.wAlgaeHolder;
import frc.robot.commands.SetPositions.Cascade.cCoralL1;
import frc.robot.commands.SetPositions.Cascade.cCoralL2;
import frc.robot.commands.SetPositions.Cascade.cCoralL3;
import frc.robot.commands.SetPositions.Cascade.cCoralL4;
import frc.robot.commands.SetPositions.Cascade.cascadeBelow0;
import frc.robot.commands.SetPositions.Wrist.wLevels;
import frc.robot.commands.SetPositions.Wrist.wZero;
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
    NamedCommands.registerCommand("L4", new cCoralL4(InitSubs.i_cascade).until(() -> InitSubs.i_cascade.m_cEncoder.getPosition()<=-150));
    NamedCommands.registerCommand("Outtake", new coralOuttake(InitSubs.i_coral));
    NamedCommands.registerCommand("StopOuttake", new coralStop(InitSubs.i_coral));
    NamedCommands.registerCommand("wAlgaeLevelIntake", new ParallelCommandGroup(new wAlgaeLevelGrab(InitSubs.i_wrist), new algaeIntake(InitSubs.i_algae)).until(() -> InitSubs.i_wrist.m_wEncoder.getPosition()>=8));
    NamedCommands.registerCommand("cAlgaeLevel1", new cAlgaeHolder(InitSubs.i_cascade).until(() -> InitSubs.i_cascade.m_cEncoder.getPosition()<=-80));
    NamedCommands.registerCommand("Reset", new ParallelCommandGroup(new wZero(InitSubs.i_wrist), new cCoralFeed(InitSubs.i_cascade)));

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
    new JoystickButton(m_driverController, 0).whileTrue(new autoTrack(InitSubs.i_lightHouse));
    new JoystickButton(m_driverController, 1).whileTrue(new autoAlign(InitSubs.i_robotDrive, false));
    new JoystickButton(m_driverController, 10).whileTrue(new cageDelift(InitSubs.i_lift));
    new JoystickButton(m_driverController,8).whileTrue(new cageLift(InitSubs.i_lift));
    new JoystickButton(m_driverController,7).whileTrue(new cageRelease(InitSubs.i_liftServo));
    //                  -- Operation Pad --  
    //new JoystickButton(m_operatorController,3).whileTrue(new algaeOuttake(InitSubs.i_algae));
    //new JoystickButton(m_operatorController,4).whileTrue(new algaeIntake(InitSubs.i_algae));
    new JoystickButton(m_operatorController,1).onTrue(new SequentialCommandGroup(new wPosition(InitSubs.i_wrist, Positions.wLevels), new cPosition(InitSubs.i_cascade, Positions.cZero), new wZero(InitSubs.i_wrist)));
    // 2,3, and 4 are coral set positions
    new JoystickButton(m_operatorController,10).onTrue(new ParallelCommandGroup(new wPosition(InitSubs.i_wrist, Positions.wAlgaeFloorIntake), new algaeIntake(InitSubs.i_algae))); 
    new JoystickButton(m_operatorController,9).onTrue(new ParallelCommandGroup(new wPosition(InitSubs.i_wrist, Positions.wAlgaeLevelGrab), new algaeIntake(InitSubs.i_algae)));
    new JoystickButton(m_operatorController,7).whileTrue(new ParallelCommandGroup(new cascadeDown(InitSubs.i_cascade), new wLevels(InitSubs.i_wrist)));
    new JoystickButton(m_operatorController,8).whileTrue(new ParallelCommandGroup(new cascadeUp(InitSubs.i_cascade), new wLevels(InitSubs.i_wrist)));
    new JoystickButton(m_operatorController,7).onFalse(new wLevels(InitSubs.i_wrist));
    new JoystickButton(m_operatorController,8).onFalse(new wLevels(InitSubs.i_wrist));
    new JoystickButton(m_operatorController,6).whileTrue(new wristDown(InitSubs.i_wrist));
    new JoystickButton(m_operatorController,5).whileTrue(new wristUp(InitSubs.i_wrist));
    new JoystickButton(m_operatorController,11).whileTrue(new coralOuttake(InitSubs.i_coral));
    new JoystickButton(m_operatorController,12).whileTrue(new algaeOuttake(InitSubs.i_algae));
    new Trigger(() -> m_operatorController.getRawAxis(0) < -.5).whileTrue(new algaeIntake(InitSubs.i_algae));
    new Trigger(() -> m_operatorController.getRawAxis(1) < -.5).onTrue(new wAlgaeProcessor(InitSubs.i_wrist));
    new Trigger(() -> m_operatorController.getRawAxis(1) < -.5).onFalse(new wAlgaeProcessor(InitSubs.i_wrist));
    
    new JoystickButton(m_operatorController,0).whileTrue(
      new ParallelCommandGroup(
        new algaeStop(InitSubs.i_algae),
        new cascadeStop(InitSubs.i_cascade),
        new coralStop(InitSubs.i_coral),
        new wristStop(InitSubs.i_wrist),
        new cageStop(InitSubs.i_lift)
      ) );
      //               -- Manual Functions--
        //Algae
          new JoystickButton(m_operatorController,0).whileTrue(new algaeOuttake(InitSubs.i_algae));
        //Coral
          new JoystickButton(m_operatorController,0).whileTrue(new coralOuttake(InitSubs.i_coral));
      //                -- Set Positions --
        //Algae

        //Algae Reef Grab
          new JoystickButton(m_operatorController,0).onTrue(
            new SequentialCommandGroup(
              new ParallelCommandGroup(
                new cAlgaeL1(InitSubs.i_cascade),
                new wLevels(InitSubs.i_wrist)),
              new wAlgaeLevelGrab(InitSubs.i_wrist)
            )
          );
          new JoystickButton(m_operatorController,0).onTrue(
            new ParallelCommandGroup(
              new cAlgaeL2(InitSubs.i_cascade),
              new wLevels(InitSubs.i_wrist)
            )
          );
        
        new JoystickButton(m_operatorController,0).onTrue(
        new ParallelCommandGroup(
          new cAlgaeHolder(InitSubs.i_cascade),
          new wAlgaeHolder(InitSubs.i_wrist),
          new algaeStop(InitSubs.i_algae)
        )
        );
        new JoystickButton(m_operatorController,0).onTrue(
        new ParallelCommandGroup(
          new cAlgaeBarge(InitSubs.i_cascade),
          new wAlgaeBarge(InitSubs.i_wrist)
        )
        );
        new JoystickButton(m_operatorController,0).onTrue(
        new ParallelCommandGroup(
          new cAlgaeFloorIntake(InitSubs.i_cascade),
          new wAlgaeFloorIntake(InitSubs.i_wrist),
          new algaeIntake(InitSubs.i_algae)
        )
        );
        new JoystickButton(m_operatorController,0).onTrue(
        new ParallelCommandGroup(
          new cAlgaeProcessor(InitSubs.i_cascade),
          new wAlgaeProcessor(InitSubs.i_wrist)
        )
        );
      //Coral
      //Coral Scorers
        new JoystickButton(m_operatorController,0).onTrue(
          new ParallelCommandGroup(
            new cCoralL1(InitSubs.i_cascade),
            new wLevels(InitSubs.i_wrist)
          )
        );
        new JoystickButton(m_operatorController,2).onTrue(
          new ParallelCommandGroup(
            new cCoralL2(InitSubs.i_cascade),
            new wPosition(InitSubs.i_wrist, Positions.wLevels)
          )
        );
        new JoystickButton(m_operatorController,3).onTrue(
          new ParallelCommandGroup(
            new cCoralL3(InitSubs.i_cascade),
            new wPosition(InitSubs.i_wrist, Positions.wLevels)
          )
        );
        new JoystickButton(m_operatorController,4).onTrue(
            new ParallelCommandGroup(
            new cCoralL4(InitSubs.i_cascade),
            new wPosition(InitSubs.i_wrist, Positions.wLevels)
            )
        );
      // Others
      new JoystickButton(m_operatorController,0).onTrue(
        new ParallelCommandGroup(
          new cCoralFeed(InitSubs.i_cascade),
          new wLevels(InitSubs.i_wrist)
        )
      );
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    
}
