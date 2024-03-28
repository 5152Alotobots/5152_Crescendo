package frc.robot.crescendo.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.crescendo.subsystems.shooter.util.PoseHashMap;

public class SubSys_Shooter_Constants {
    
    public static final class ShooterArm {
        public static final double CANcoderMagOffset = 0.10644;//0.2708; //-0.722;

        public static final class SoftwareLimits {

            public static final double SHOOTER_LIMIT_FORWARD = 0; // Towards intake
            public static final double SHOOTER_LIMIT_REVERSE = -0.583; // Towards climber
            public static final boolean SHOOTER_LIMIT_ENABLE = true;
        }
    }

    public static final class ShooterWheels{

        public static final double OpenLoopRampRate = 0.5; // seconds

        public static final class PID {
            public static final double Pgain = 2.5;
            public static final double Igain = 0.0;
            public static final double Dgain = 0.0;
            public static final double Izone = 0.0;
            public static final double Vgain = 0.125;
            public static final double MaxOutput = 1;
            public static final double MinOutput = -1;
            //public static final double MaxRPM = 5700;
            public static final double ClosedLoopRampRate = 0.5; // seconds
        }

        public static final class SpeedSetPoints{
            public static final double SPEAKER_DEFAULT_SPD_CMD = 90.0; // rotations/s
        }
    }

    public static final class ShooterRoller{

        public static final double OpenLoopRampRate = 0.25; // seconds

        public static final class PID {
            public static final double Pgain = 0.0;
            public static final double Igain = 0.0;
            public static final double Dgain = 0.0;
            public static final double Izone = 0.0;
            public static final double FFwd = 0.0;
            public static final double MaxOutput = 1;
            public static final double MinOutput = -1;
            public static final double MaxRPM = 5700;
            public static final double ClosedLoopRampRate = 0.0; // seconds
        }

        public static final class SpeedSetPoints{
            
        }
    }

    public static final class PID {
        static final class Shooter {
            public static final double SHOOTER_P = 0.1;
            public static final double SHOOTER_I = 0;
            public static final double SHOOTER_D = 0;
            public static final double SHOOTER_IZONE = 1;
            public static final double SHOOTER_RAMP_RATE = 0.5; // Number of seconds to full speed
            public static final double SHOOTER_V = 0.125; // Feedforward gain.
        }

        static final class Arm {
            public static final double ARM_P = 40;
            public static final double ARM_I = 0;
            public static final double ARM_D = 0;

        }
    }
    public static final class Speeds {
        public static final double MAX_ARM_ROTATION_SPEED = 0.30;
        public static final double MAX_SHOOTER_SPEED = 1;
        public static final double MAX_SHOOTER_SPPED_MPS = 6.2;
        public static final double MAX_INTAKE_SPEED = 1;
        public static final double TRANSFER_INTAKE_SPEED = 0.3;
        public static final double SHOOTER_AMP_SPEED = 0.3;
    }

    public static final class AutoAim {
        public static final double METERS_TO_RPM_RATIO = 1200; // X number of rotations per minute equals 1 meter per second

        public static final double LAUNCH_TOLERANCE = 1; // +- 1 Meter to launch
        public static final double SHOOT_SPIN_UP_TEMP = .75; // Wait before launch
        public static final double SHOOTER_VELOCITY_TOLERANCE = 0.5; // +-X is counted as ready to shoot
        public static final double SHOOTER_ARM_POSITION_TOLERANCE = 0.02; // +-X counts as at position
        public static final double SHOOTER_WAIT_AFTER_SHOOT = 0.5; // X seconds
        public static final double READY_TO_SHOOT_RPS = 65; // >85rps is ready to shoot
    }

    public static final class FieldConstants {
        public static final Pose2d SPEAKER_XY = new Pose2d(15.55, 5.45, new Rotation2d(0));

    }

    public static final class PresentArmPositions {
        public static final double ARM_PRESET_TRANSFER = -130;
        public static final double ARM_PRESET_AMP = -205;
        public static final double ARM_PRESET_SPEAKER = -125;
    }

    public static final class ControllerOptions {
        public static final double DEADBAND_PERCENT = 0.1;
    }

    public static final Pose2d SPEAKER_POSE_BLUE = new Pose2d(1, 5.4, Rotation2d.fromDegrees(180));
    public static final Pose2d SPEAKER_POSE_RED = new Pose2d(15.6, 5.4, Rotation2d.fromDegrees(0));
    // This is an example of the PoseHashMap class. It should have a pose value from the blue side and a given shooter angle.
    // Rotational comparison is not taken into account yet
    public static final PoseHashMap ANGLE_BY_DISTANCE_BLUE = new PoseHashMap(){{
        // METERS
        // DIST  DEG
        put(new Pose2d(1.862, 5.38, new Rotation2d()), -130.9); // right next to speaker, center
        put(new Pose2d(2.8, 4.075, new Rotation2d()), -150.1); // just in front of rightmost close note
        put(new Pose2d(2.8, 5.43, new Rotation2d()), -147.2); // just in front of center close note
        put(new Pose2d(2.8, 6.78, new Rotation2d()), -147.2); // just in front of leftmost close note
        put(new Pose2d(2.325, 4.82, new Rotation2d()), -142.7); // In between right and center close note, center on white line
        put(new Pose2d(2.325, 6.15, new Rotation2d()), -147.53); // In between left and center close note, center on white line
    }};

    public static final PoseHashMap ANGLE_BY_DISTANCE_RED = ANGLE_BY_DISTANCE_BLUE.mirrorForRedSide();
}
