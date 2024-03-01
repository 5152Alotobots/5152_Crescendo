package frc.robot.crescendo.subsystems.intake;

public class SubSys_Intake_Constants {
    public static final class IntakeArm {
        public static final double CANcoderMagOffset = -0.173;

        public static final double FwdLimitSwitchPos = 0;       // On the Ground
        public static final double RevLimitSwitchPos = 0.3;     // Up in the air
        public static final double IntakeArmPickupPos = 0.0;    // Note Pickup Position
        public static final double IntakeArmStowPos = 0.25;     // Stow Position
        public static final double IntakeArmTransferPos = 0.25; // Transfer Position
        public static final double IntakeArmPosCmdSpd = 0.15;   // -1 - +1

        static final class Arm {
            public static final double ARM_P = 12;
            public static final double ARM_I = 11;
            public static final double ARM_D = 1;
        }
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

        public static final double OpenLoopRampRate = 0; // seconds

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

    public static final class PresetIntakePositions {
        public static final double INTAKE_PRESET_TRANSFER = -150;
        public static final double INTAKE_PRESET_PICKUP = 10;
        public static final double INTAKE_PRESET_STOW = -100;
        public static final double INTAKE_ARM_POSITION_TOLERANCE = 0.01; // +-X counts as at position
    }

    public static final class Limit {
        public static final double ARM_LIMIT_FORWARD = 0.08;
        public static final double ARM_LIMIT_REVERSE = -0.43;
        public static final boolean ARM_LIMIT_ENABLE = true;
    }

}
