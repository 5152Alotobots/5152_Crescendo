package frc.robot.crescendo.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SubSys_Shooter_Constants {
    
    public static final class ShooterArm {
        public static final double CANcoderMagOffset = .1026;

        public static final double ShooterArmTransferPos = 45;
        public static final double ShooterArmAmpPos = 45;
    }

    public static final class ShooterWheels{

        public static final double OpenLoopRampRate = 0.5; // seconds

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

        public static final class SpeedSetPoints{

        }
    }

    public static final class ShooterRoller{

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

        public static final class SpeedSetPoints{
            
        }
    }

    public static final class PID {
        static final class Shooter {
            public static final double SHOOTER_P = 0;
            public static final double SHOOTER_I = 0;
            public static final double SHOOTER_D = 0;
            public static final double SHOOTER_IZONE = 1;
            public static final double SHOOTER_RAMP_RATE = 1; // Number of seconds to full speed
        }

        static final class Arm {
            public static final double ARM_P = 40;
            public static final double ARM_I = 0;
            public static final double ARM_D = 0;

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
        public static final double METERS_TO_RPM_RATIO = 1200; // X number of rotations per minute equals 1 meter per second

        public static final double LAUNCH_TOLERANCE = 1; // +- 1 Meter to launch
        public static final double SHOOT_SPIN_UP_TEMP = 1.5;
        public static final double SHOOTER_VELOCITY_TOLERANCE = 0.5; // +-X is counted as ready to shoot
        public static final double SHOOTER_ARM_VELOCITY_TOLERANCE = 0.02; // +-0.5 counts as not busy/ready to shoot
    }

    public static final class FieldConstants {
        public static final Pose2d SPEAKER_XY = new Pose2d(15.55, 5.45, new Rotation2d(0));

    }

    public static final class PresentArmPositions {
        public static final double ARM_PRESET_TRANSFER = -135;
    }

    public static final class ControllerOptions {
        public static final double DEADBAND_PERCENT = 0.1;
    }
}
