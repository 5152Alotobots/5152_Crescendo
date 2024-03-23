package frc.robot.library.vision.limelight;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class SubSys_Limelight_Constants {
    // FORWARD: +, RIGHT: +, UP: + (USED FOR Object Detection)
    // ROLL, PITCH, YAW: CCW +
    public static final Transform3d LL_OFFSET =
            new Transform3d(
                    new Translation3d(0.23, 0, 0.42),
                    new Rotation3d(0, Math.toRadians(12), Math.toRadians(0)));

    public static final boolean OBJECT_DETECTION_ENABLED = true;
    public static final String NN_LIMELIGHT = "limelight";
    public static final double CONFIDENCE_DECAY = 0.1 * 0.02; // decay per second (10 seconds to decay assuming 20ms loop time)
    public static final double POSE_TOLERANCE = 0.15; // 15cm
}
