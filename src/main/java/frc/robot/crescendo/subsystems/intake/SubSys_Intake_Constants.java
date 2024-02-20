package frc.robot.crescendo.subsystems.intake;

public class SubSys_Intake_Constants {
    public static final class IntakeArm {
        public static final double CANcoderMagOffset = 0.0;
    }

    public static final class MaxSpeeds {
        public static final double MAX_ARM_ROTATION_SPEED = 0.15;
        public static final double MAX_INTAKE_SPEED = -1;
        public static final double SLOW_OUTTAKE_SPEED = 0.2;
    }

    public static final class IntakeRoller {
        public static final double intakeSpeed = 0.75;
        public static final double deploySpeed = 0.25;
    }
}
