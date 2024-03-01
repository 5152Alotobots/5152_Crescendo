package frc.robot.crescendo.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SubSys_Shooter_Constants {
    
    public static final class ShooterArm {
        public static final double CANcoderMagOffset = -0.9; // -.89;

        public static final double FwdSWLimitPos = -0.25;   // Revs Straight Up
        public static final double RevSWLimitPos = -0.65;   // Revs Downward

        public static final double ShooterArmTransferPos = -0.325;  // Revs Transfer Position
        
        public static final double ShooterArmAmpPos = -0.6;         // Revs Amp Scoring Position

        public static final double ShooterArmSpeakerLeftPos = -0.325; // Revs Speaker Left Scoring Position
        public static final double ShooterArmSpeakerCenterPos = -0.325; // Revs Speaker Center Scoring Position
        public static final double ShooterArmSpeakerRightPos = -0.325; // Revs Speaker Right Scoring Position

        public static final double ShooterArmPosCmdFastDutyCycle = 0.7; // 0-1
        public static final double ShooterArmPosCmdSlowDutyCycle = 0.2; // 0-1
    }

    public static final class ShooterWheels{

        public static final double OpenLoopRampRate = 0.5; // seconds

        public static final double ShootSpeakerDutyCycleCmd = 1; // -1 - 1
        public static final double ShootAmpDutyCycleCmd = 0.75; // -1 - 1
        public static final double IntakeNoteDutyCycleCmd = -0.7; // -1 - 1

        public static final class PID {
            public static final double Pgain = 0.0;
            public static final double Igain = 0.0;
            public static final double Dgain = 0.0;
            public static final double Izone = 0.0;
            public static final double FFwd = 0.0;
            public static final double MaxOutput = 1;
            public static final double MinOutput = -1;
            public static final double MaxRPM = 5700;
            public static final double ClosedLoopRampRate = 0.5; // seconds
        }
    }

    public static final class ShooterRoller{

        public static final double OpenLoopRampRate = 0.25; // seconds
        public static final double TransferNoteDutyCycle = -0.5;  // -1 - 1
        public static final double EjectNoteDutyCycle = 0.5;  // -1 - 1
        public static final double ShootNoteDutyCycle = -1.0;  // -1 - 1
        public static final double SourceIntakeNoteDutyCycle = 0.25;  // -1 - 1
        

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

        public static final class SpeedSetPoints{
            
        }
    }

    public static final class MaxSpeeds {
        public static final double MAX_ARM_ROTATION_SPEED = 0.15;
        public static final double MAX_SHOOTER_SPEED = 1;
        public static final double MAX_SHOOTER_SPPED_MPS = 6.2;
        public static final double MAX_INTAKE_SPEED = 1;
        public static final double TRANSFER_INTAKE_SPEED = 0.5;
    }

    public static final class AutoAim {

        public static final double LAUNCH_TOLERANCE = 1; // +- 1 Meter to launch
        public static final double SHOOT_SPIN_UP_TEMP = 1.5;
    }

    public static final class FieldConstants {
        public static final Pose2d SPEAKER_XY = new Pose2d(15.55, 5.45, new Rotation2d(0));

    }
}
