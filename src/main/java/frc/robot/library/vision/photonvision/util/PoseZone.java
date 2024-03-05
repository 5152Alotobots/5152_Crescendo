package frc.robot.library.vision.photonvision.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class PoseZone {
    private final Pose2d lowerLeftBound;
    private final Pose2d upperRightBound;

    public PoseZone(
            Pose2d lowerLeftBound,
            Pose2d upperRightBound) {
        this.lowerLeftBound = lowerLeftBound;
        this.upperRightBound = upperRightBound;
    }

    public Pose2d getLowerLeftBound() {
        return lowerLeftBound;
    }

    public Pose2d getUpperRightBound() {
        return upperRightBound;
    }

    /**
     * @param pose The {@link Pose3d} reference
     * @return If the pose is inside the zone
     */
    public boolean isInside(Pose3d pose) {
        Pose2d lowerLeft = getLowerLeftBound();
        Pose2d upperRight = getUpperRightBound();
        boolean xInside = pose.getX() > lowerLeft.getX() && pose.getX() < upperRight.getX();
        boolean yInside = pose.getY() > lowerLeft.getY() && pose.getY() < upperRight.getY();
        return xInside && yInside;
    }

    public boolean isInside(Pose2d pose) {
        Pose2d lowerLeft = getLowerLeftBound();
        Pose2d upperRight = getUpperRightBound();
        boolean xInside = pose.getX() > lowerLeft.getX() && pose.getX() < upperRight.getX();
        boolean yInside = pose.getY() > lowerLeft.getY() && pose.getY() < upperRight.getY();
        return xInside && yInside;
    }
}
