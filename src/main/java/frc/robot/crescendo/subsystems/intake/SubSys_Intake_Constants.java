package frc.robot.crescendo.subsystems.intake;

public class SubSys_Intake_Constants {
    public static final class IntakeArm {
        public static final double CANcoderMagOffset = -0.820068;

        public static final double FwdLimitSwitchPos = 0;       // On the Ground
        public static final double RevLimitSwitchPos = 0.3;     // Up in the air
        public static final double IntakeArmPickupPos = 0.0;    // Note Pickup Position
        public static final double IntakeArmStowPos = 0.25;     // Stow Position
        public static final double IntakeArmTransferPos = 0.25; // Transfer Position
        public static final double IntakeArmPosCmdSpd = 0.15;   // -1 - +1
    }

    public static final class MaxSpeeds {
        public static final double MAX_ARM_ROTATION_SPEED = 0.35; //0.15;
        public static final double MAX_INTAKE_SPEED = -1;
        public static final double TRANSFER_SPEED = 0.2;
    }

    public static final class IntakeRoller {
        public static final double intakeSpeed = 0.75;
        public static final double deploySpeed = 0.25;

        public static final double intakeNoteSpeed = -0.60;      // -1 - +1
        public static final double ejectNoteSpeed = 0.25;       // -1 - +1
        public static final double transferNoteSpeed = 0.25;    // -1 - +1

        public static final double OpenLoopRampRate = 1; // seconds

        public static final class PID {
            public static final double Pgain = 0.0;
            public static final double Igain = 0.0;
            public static final double Dgain = 0.0;
            public static final double Izone = 0.0;
            public static final double FFwd = 0.0;
            public static final double MaxOutput = 1;
            public static final double MinOutput = -1;
            public static final double MaxRPM = 5700;
            public static final double ClosedLoopRampRate = 1; // seconds

        }
    }


}
