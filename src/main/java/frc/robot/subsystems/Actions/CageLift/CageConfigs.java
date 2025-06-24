package frc.robot.subsystems.Actions.CageLift;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class CageConfigs {
    public static final class CageLiftConfigs {
        public static final SparkMaxConfig cageConfig = new SparkMaxConfig();

        static {
            cageConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(80);
            cageConfig.encoder
                    .positionConversionFactor(1) // meters
                    .velocityConversionFactor(1); // meters per second
            cageConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    //position PIDS
                    .p(.1)
                    .d(.1)
                    .outputRange(-1, 1);

        }
    }
}



