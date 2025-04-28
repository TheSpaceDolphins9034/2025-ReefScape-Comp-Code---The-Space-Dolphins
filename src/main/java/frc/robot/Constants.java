// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.subsystems.Actions.Algae;
import frc.robot.subsystems.Actions.Coral;
import frc.robot.subsystems.Actions.LEDController;
import frc.robot.subsystems.Actions.Lift;
import frc.robot.subsystems.Actions.LiftServo;
import frc.robot.subsystems.Actions.Wrist;
import frc.robot.subsystems.Actions.Cascade.Cascade;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Drive.LightHouse;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //These are the initialization of Kepler's ID's
    //motors
      public static final int CascadeID = 9;
      public static final int CascadeID2 = 4;
      public static final int CoralID = 8;
      public static final int AlgaeID = 7;
      public static final int LiftID = 6;
      public static final int WristID = 5;
      public static final int ServoReleaseID = 0;
    //sensors
      public static final int coralSwitchID = 2;
      public static final int ultraSonicID = 4;

    //These are the initialization of Kepler's motors
    public static final class Motors{
      //public static final SparkMax m_cascade = new SparkMax(Constants.CascadeID, MotorType.kBrushless);
      //public static final SparkMax m_cascade2 = new SparkMax(Constants.CascadeID2, MotorType.kBrushless);
      public static final SparkMax m_coral = new SparkMax(Constants.CoralID, MotorType.kBrushless);
      public static final SparkMax m_algae = new SparkMax(Constants.AlgaeID, MotorType.kBrushless);
      public static final SparkMax m_lift = new SparkMax(Constants.LiftID, MotorType.kBrushless);
      public static final SparkMax m_wrist = new SparkMax(Constants.WristID, MotorType.kBrushless);
      public static final Servo m_release = new Servo(ServoReleaseID);
    }
    //These are the initialization of Kepler's sensors
    public static final class Sensors{
      public static final DigitalInput m_coralSwitch = new DigitalInput(coralSwitchID);
      public static final AnalogInput m_ultraSonic = new AnalogInput(ultraSonicID);
      public static final AddressableLED m_led = new AddressableLED(1);
      public static final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(300);
    }

    //These are the initialization of Kepler's subsystems
    public static final class InitSubs{
      public static Algae i_algae = new Algae();
      public static Cascade i_cascade = new Cascade();
      public static Coral i_coral = new Coral();
      public static Lift i_lift = new Lift();
      public static Wrist i_wrist = new Wrist();
      public static LightHouse i_lightHouse = new LightHouse();
      public static DriveSubsystem i_robotDrive = new DriveSubsystem();
      public static LiftServo i_liftServo = new LiftServo();
      public static LEDController i_ledContronller = new LEDController();
    }

    public static class CascadeIO{
      public static final boolean IS_INVERTED = false;
      public static final double ELEVATOR_PEAK_CURRENT_LIMIT = 60.0;
      public static final double ELEVATOR_PEAK_MAXHEIGHT_LIMIT = 185.0;
      public static final double ELEVATOR_PEAK_MINHEIGHT_LIMIT = -1;
    }

    public static final class Positions {
      /*
       *        Cascade Positions
       *        Min: 0  Max: -185
      */
      public static final double cZero = 0;
      public static final double cCoralL1 = -45;
      public static final double cCoralL2 = -75;
      public static final double cCoralL3 = -105;
      public static final double cCoralL4 = -155.5;
      public static final double cAlgaeL1 = 17;
      public static final double cAlgaeL2 = 17;
      public static final double cBarge = 17;

      /*
       *         Wrist Postions
       *        Min: 0  Max: -11
      */
      public static final double wZero = 0;
      public static final double wLevels = 6;
      public static final double wAlgaeLevelGrab = 17;
      public static final double wAlgaeBarge = 7;
      public static final double wAlgaeFloorIntake = 11;
      public static final double wAlgaeProcessor = 7.3;
      public static final double wAlgaeHolder = 17;
      public static final double wAlgaeLolipop = 17;
    }

    public static final class DriveConstants {
      // SPARK MAX CAN IDs
      public static final int kFrontLeftDrivingCanId = 17; //11
      public static final int kRearLeftDrivingCanId = 15; //13
      public static final int kFrontRightDrivingCanId = 13; //15
      public static final int kRearRightDrivingCanId = 11; //17
 
      public static final int kFrontLeftTurningCanId = 16; //10
      public static final int kRearLeftTurningCanId = 14; //12
      public static final int kFrontRightTurningCanId = 12; //14
      public static final int kRearRightTurningCanId = 10; //16

      // Driving Parameters - Note that these are not the maximum capable speeds of
      // the robot, rather the allowed maximum speeds
      public static final double kMaxSpeedMetersPerSecond = 4.8;
      public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

      public static final double kDirectionSlewRate = 1.2; // radians per second
      public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
      public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

      // Chassis configuration
      public static final double kTrackWidth = Units.inchesToMeters(26.5);
      // Distance between centers of right and left wheels on robot
      public static final double kWheelBase = Units.inchesToMeters(26.5);
      // Distance between front and back wheels on robot
      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

      // Angular offsets of the modules relative to the chassis in radians
      public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
      public static final double kFrontRightChassisAngularOffset = 0;
      public static final double kBackLeftChassisAngularOffset = Math.PI;
      public static final double kBackRightChassisAngularOffset = Math.PI / 2;

      public static final boolean kGyroReversed = false;
    }
    public static final class Swerve {
      public static final Translation2d flModuleOffset = new Translation2d(0.546 / 2.0, 0.546 / 2.0);
      public static final Translation2d frModuleOffset = new Translation2d(0.546 / 2.0, -0.546 / 2.0);
      public static final Translation2d blModuleOffset = new Translation2d(-0.546 / 2.0, 0.546 / 2.0);
      public static final Translation2d brModuleOffset = new Translation2d(-0.546 / 2.0, -0.546 / 2.0);

      public static final double maxModuleSpeed = 4.5; // M/S

      public static final PIDConstants translationConstants = new PIDConstants(5.0, 0.0, 0.0);
      public static final PIDConstants rotationConstants = new PIDConstants(5.0, 0.0, 0.0);
    }
    public static final class ModuleConstants {
      // The MAXSwerve module can be configured with one of three pinion gears: 12T,
      // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
      // more teeth will result in a robot that drives faster). (Caiden was here)
      public static final int kDrivingMotorPinionTeeth = 13;

      // Calculations required for driving motor conversion factors and feed forward
      public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
      public static final double kWheelDiameterMeters = 0.0762;
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
      // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
      // teeth on the bevel pinion
      public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
      public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
          / kDrivingMotorReduction;
    }

    public static final class OIConstants {
    
      public static final int kDriverControllerPort = 0;
      public static final int kOperatorControllerPort = 1;
      public static final double kDriveDeadband = 0.05;
    }

    public static final class AutoConstants {
      public static final double kMaxSpeedMetersPerSecond = 3;
      public static final double kMaxAccelerationMetersPerSecondSquared = 3;
      public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

      public static final double kPXController = 1;
      public static final double kPYController = 1;
      public static final double kPThetaController = 1;

      // Constraint for the motion profiled robot angle controller
      public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
          kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
      public static final double kFreeSpeedRpm = 5676;
    }
}

