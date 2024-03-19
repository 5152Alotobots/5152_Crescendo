package frc.robot.library.vision.limelight;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class SubSys_Limelight_Constants {
    // FORWARD: +, RIGHT: +, UP: + (USED FOR Object Detection)
    public static final Transform3d LL_OFFSET =
            new Transform3d(
                    new Translation3d(-0.36, 0.175, 0.172),
                    new Rotation3d(0, Math.toRadians(54), Math.toRadians(180)));

    public static final boolean OBJECT_DETECTION_ENABLED = true;
    public static final String NN_LIMELIGHT = "limelight";
}
