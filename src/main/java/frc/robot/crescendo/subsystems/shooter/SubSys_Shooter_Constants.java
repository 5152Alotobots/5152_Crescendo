package frc.robot.crescendo.subsystems.shooter;

public class SubSys_Shooter_Constants {
    
    public static final class ShooterArm {
        public static final double CANcoderMagOffset = -0.316162;
    }

    public static final class ShooterRoller {
        public static final double shooterSpeed = 0.5;
        public static final double deploySpeed = 0.25;
    }

    public static final class MaxSpeeds {
        public static final double MAX_ARM_ROTATION_SPEED = 0.4;
    }

    public static final class Controller {
        public static final double ARM_ROTATION_DEADBAND = 0.5;
        public static final double ARM_ROTATION_SCALE_FACTOR = 0.25;

    }
}
