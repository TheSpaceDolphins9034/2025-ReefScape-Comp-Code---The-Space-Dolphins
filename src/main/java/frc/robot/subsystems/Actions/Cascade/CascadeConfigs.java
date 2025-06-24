package frc.robot.subsystems.Actions.Cascade;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.Motors;

public final class CascadeConfigs {
    public static final class Cascade {
        public static final SparkMaxConfig cLeadConfig = new SparkMaxConfig();
        public static final SparkMaxConfig cFollowerConfig = new SparkMaxConfig();

        static {
 
            //Cascade Motor 1
            cLeadConfig
                .idleMode(IdleMode.kBrake);

            cLeadConfig.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);

            cLeadConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                //position PIDS
                .p(.1)
                .d(1)
                .outputRange(-1, .75);

            //Cascade Motor 2
                cFollowerConfig.follow(Motors.m_cLead);

                cFollowerConfig.idleMode(IdleMode.kBrake);
   
                cFollowerConfig.encoder
                    .positionConversionFactor(1)
                    .velocityConversionFactor(1);

                cFollowerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    //position PIDS
                    .p(.1)
                    .d(1)
                    .outputRange(-1, .75);
        }
    }
}
