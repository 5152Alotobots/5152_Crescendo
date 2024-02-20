package frc.robot.crescendo.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SubSys_Shooter_Constants {
    
    public static final class ShooterArm {
        public static final double CANcoderMagOffset = -.498125;
    }

    public static final class MaxSpeeds {
        public static final double MAX_ARM_ROTATION_SPEED = 0.15;
        public static final double MAX_SHOOTER_SPEED = 1;
        public static final double MAX_INTAKE_SPEED = 1;
        public static final double SLOW_INTAKE_SPEED = 0.4;
    }

    public static final class FieldConstants {
        public static final Pose2d SPEAKER_XY = new Pose2d(15.55, 5.45, new Rotation2d(0));
    }
}
