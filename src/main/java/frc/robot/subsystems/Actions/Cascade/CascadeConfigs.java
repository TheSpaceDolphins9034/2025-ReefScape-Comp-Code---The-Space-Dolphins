package frc.robot.subsystems.Actions.Cascade;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CascadeIO;

public final class CascadeConfigs {
    public static final class Cascade {
        public static final TalonFXConfiguration leadConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        static {
                leadConfig.Voltage.PeakReverseVoltage = -7.8;
                leadConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                leadConfig.CurrentLimits.SupplyCurrentLimit = CascadeIO.ELEVATOR_PEAK_CURRENT_LIMIT;
                leadConfig.CurrentLimits.SupplyCurrentLowerLimit = CascadeIO.ELEVATOR_PEAK_CURRENT_LIMIT;
                leadConfig.CurrentLimits.SupplyCurrentLowerTime = 0;
                leadConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                leadConfig.CurrentLimits.StatorCurrentLimit = CascadeIO.ELEVATOR_PEAK_CURRENT_LIMIT;
                leadConfig.CurrentLimits.StatorCurrentLimitEnable = true;
                leadConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = CascadeIO.ELEVATOR_PEAK_MAXHEIGHT_LIMIT;
                leadConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
                leadConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = CascadeIO.ELEVATOR_PEAK_MINHEIGHT_LIMIT;
                leadConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

                leadConfig.MotorOutput.Inverted =
                    CascadeIO.IS_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

                leadConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

                leadConfig.Slot0.kP = 1.1;
                leadConfig.Slot0.kI = 0;
                leadConfig.Slot0.kD = 0.13;
                leadConfig.Slot0.kS = 0;
                leadConfig.Slot0.kV = 0;
                leadConfig.Slot0.kA = 0;
                leadConfig.Slot0.kG = 0;

                leadConfig.Slot0.withGravityType(GravityTypeValue.Elevator_Static);

                leadConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
                leadConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;
                leadConfig.HardwareLimitSwitch.ReverseLimitEnable = true;


                followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

                followerConfig.CurrentLimits.SupplyCurrentLimit = CascadeIO.ELEVATOR_PEAK_CURRENT_LIMIT;
                followerConfig.CurrentLimits.SupplyCurrentLowerLimit = CascadeIO.ELEVATOR_PEAK_CURRENT_LIMIT;
                followerConfig.CurrentLimits.SupplyCurrentLowerTime = 0;
                followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                followerConfig.CurrentLimits.StatorCurrentLimit = CascadeIO.ELEVATOR_PEAK_CURRENT_LIMIT;
                followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        }
    }
}
