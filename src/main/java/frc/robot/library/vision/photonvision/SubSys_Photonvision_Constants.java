package frc.robot.library.vision.photonvision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class SubSys_Photonvision_Constants {

  // FORWARD: +, RIGHT: +, UP: + (USED FOR APRILTAGS)
  public static final Transform3d CAMERA_OFFSET =
      new Transform3d(
              new Translation3d(-0.36, 0.175, 0.172),
              new Rotation3d(0, 54, Math.toRadians(180)));
  public static final boolean USE_VISION_POSE_ESTIMATION = false;
}
