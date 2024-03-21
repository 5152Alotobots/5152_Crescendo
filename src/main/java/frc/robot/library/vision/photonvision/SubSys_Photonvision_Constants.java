package frc.robot.library.vision.photonvision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class SubSys_Photonvision_Constants {

  // FORWARD: +, RIGHT: +, UP: + (USED FOR APRILTAGS)
  public static final Transform3d CAMERA_OFFSET =
      new Transform3d(
              new Translation3d(-0.36, 0.175, 0.172),
              new Rotation3d(0, 54, Math.toRadians(180)));
  public static final boolean USE_VISION_POSE_ESTIMATION = false;


  /*
   * The standard deviations to use for the vision
   */
  // NEEDS TO BE COPIED FROM PHOTONVISION ON COMPETITION DAY!!
  public static final Matrix<N3, N1> VISION_STD_DEVS = MatBuilder.fill(Nat.N3(), Nat.N1(),
          0.00443, // x in meters (default=0.9)
          0.00630, // y in meters (default=0.9)
          1000  // heading in radians. The gyroscope is very accurate, so as long as it is reset correctly it is unnecessary to correct it with vision
  );
}
