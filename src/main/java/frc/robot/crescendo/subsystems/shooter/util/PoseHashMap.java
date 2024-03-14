package frc.robot.crescendo.subsystems.shooter.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

public class PoseHashMap extends HashMap<Pose2d, Double> {
    private static final Comparator<Pose2d> POSE_COMPARATOR = Comparator.comparingDouble(
        pose -> pose.getTranslation().getNorm()
    );

    public Double getValueForClosestPose(Pose2d target) {
        // Create a TreeMap from this HashMap with a custom Comparator for Pose2d
        TreeMap<Pose2d, Double> sortedMap = new TreeMap<>(POSE_COMPARATOR);
        sortedMap.putAll(this);

        // Find the closest pose
        Pose2d closestPose;
        if (sortedMap.containsKey(target)) {
            closestPose = target;
        } else {
            Pose2d floorPose = sortedMap.floorKey(target);
            Pose2d ceilingPose = sortedMap.ceilingKey(target);

            double floorDistance = Double.MAX_VALUE;
            double ceilingDistance = Double.MAX_VALUE;

            if (floorPose != null) {
                floorDistance = floorPose.getTranslation().getDistance(target.getTranslation());
            }

            if (ceilingPose != null) {
                ceilingDistance = ceilingPose.getTranslation().getDistance(target.getTranslation());
            }

            closestPose = (floorDistance < ceilingDistance) ? floorPose : ceilingPose;
        }

        // Get the value for the closest pose from this HashMap
        return this.get(closestPose);
    }
}