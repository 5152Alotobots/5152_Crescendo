package frc.robot.library.vision.photonvision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class SubSys_Photonvision_Constants {

  // FORWARD: +, RIGHT: +, UP: + (USED FOR APRILTAGS)
  public static final Transform3d cameraOffset =
      new Transform3d(
          new Translation3d(0.2, 0.105, 0.47),
              new Rotation3d(0, 0, 0));

}
