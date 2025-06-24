package frc.robot.subsystems.Actions.GamePieceManipulators;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class ManipulatorConfigs {
    public static final class GameManipulators {
        public static final SparkMaxConfig coralConfig = new SparkMaxConfig();
        public static final SparkMaxConfig algaeConfig = new SparkMaxConfig();

        static {
            coralConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);

            algaeConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
        }
    }
}
