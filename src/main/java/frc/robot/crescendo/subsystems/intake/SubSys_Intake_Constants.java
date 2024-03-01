package frc.robot.crescendo.subsystems.intake;

public class SubSys_Intake_Constants {
    public static final class IntakeArm {
        public static final double CANcoderMagOffset = 0.173;

        public static final double FwdLimitSwitchPos = 0;       // Revs On the Ground
        public static final double RevLimitSwitchPos = -0.3;    // Revs Up in the air

        public static final double FwdSWLimitPos = 0;           // Revs on the Ground
        public static final double RevSWLimitPos = -0.3;        // Revs Up in the air

        public static final double IntakeArmPickupPos = 0.0;    // Revs Note Pickup Position
        public static final double IntakeArmStowPos = -0.25;    // Revs Stow Position
        public static final double IntakeArmTransferPos = 0.25; // Revs Transfer Position

        public static final double IntakeArmPosCmdFastDutyCycle = 0.5;   // -1 - +1
        public static final double IntakeArmPosCmdSlowDutyCycle = 0.15;   // -1 - +1

        public static final double MaxArmDutyCycle = 0.4;  // 0-1
    }

    public static final class MaxSpeeds {
        public static final double MAX_ARM_ROTATION_SPEED = 0.35; //0.15;
        public static final double MAX_INTAKE_SPEED = -1;
        public static final double TRANSFER_SPEED = 0.2;
    }

    public static final class IntakeRoller {
        public static final double intakeSpeed = 0.75;
        public static final double deploySpeed = 0.25;

        public static final double intakeNoteDutyCycle = -0.75;      // -1 - +1
        public static final double ejectNoteDutyCycle = 0.25;       // -1 - +1
        public static final double transferNoteDutyCycle = 0.25;    // -1 - +1

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


}
