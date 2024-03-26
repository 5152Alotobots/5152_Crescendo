package frc.robot.library.vision.limelight.util;

import edu.wpi.first.math.geometry.*;

import java.util.ArrayList;
import java.util.Comparator;

import static frc.robot.library.vision.limelight.SubSys_Limelight_Constants.*;

/**
 * A list implementation that extends ArrayList to store DetectedObjectPair objects.
 * The list automatically decreases the confidence value of each entry by a fixed amount (CONFIDENCE_DECAY) at every update.
 * Entries with a confidence value of 0.0 or less are removed from the list.
 */
public class DetectedObjectList extends ArrayList<DetectedObjectList.DetectedObjectPair> {

    /**
     * Adds a new DetectedObjectPair to the list.
     * If the new DetectedObject's pose is within a certain range (POSE_TOLERANCE) of an existing pose,
     * the existing DetectedObject is updated instead of adding a new one.
     *
     * @param pair The DetectedObjectPair to be added or updated in the list.
     */
    @Override
    public boolean add(DetectedObjectPair pair) {
        for (DetectedObjectPair existingPair : this) {
            DetectedObject existingObject = existingPair.getObject();
            DetectedObject newObject = pair.getObject();

            if (isPoseWithinTolerance(existingObject.pose, newObject.pose, POSE_TOLERANCE)) {
                // Update the existing DetectedObject with the new confidence and return
                existingPair.setObject(newObject);
                existingPair.setConfidence(pair.getConfidence());
                return true;
            }
        }

        // If no existing DetectedObject within the pose tolerance is found, add a new entry
        return super.add(pair);
    }

    /**
     * Updates the list by decreasing the confidence value of each entry by CONFIDENCE_DECAY.
     * If the updated confidence value of an entry is 0.0 or less, the entry is removed from the list.
     */
    public void update() {
        removeIf(pair -> {
            double updatedConfidence = pair.getConfidence() - CONFIDENCE_DECAY;
            if (updatedConfidence <= 0.0) {
                return true;
            } else {
                pair.setConfidence(updatedConfidence);
                return false;
            }
        });
    }

    /**
     * Sorts the list based on the confidence values, with the highest confidence value at index 0 and decreasing as the index increases.
     * Updates the list and returns the new copy.
     *
     * @return The sorted list of DetectedObjectPair objects.
     */
    public DetectedObjectList sortByConfidence() {
        sort(Comparator.comparingDouble(DetectedObjectPair::getConfidence).reversed());
        return this;
    }

    /**
     * Sorts the list based on the distance from the given Pose2d to the Pose3d of each DetectedObject.
     * The DetectedObject with the closest Pose3d to the given Pose2d will be at index 0, and the distance increases as the index increases.
     * Updates the list and returns the new copy.
     *
     * @param pose The Pose2d to calculate distances from.
     * @return The sorted list of DetectedObjectPair objects.
     */
    public DetectedObjectList sortByPose(Pose2d pose) {
        sort(Comparator.comparingDouble(pair -> {
            Translation2d translation = new Translation2d(pair.getObject().pose.getX(), pair.getObject().pose.getY());
            return pose.getTranslation().getDistance(translation);
        }));
        return this;
    }

    /**
     * Checks if two Pose3d objects are within a certain tolerance range.
     *
     * @param pose1     The first Pose3d object.
     * @param pose2     The second Pose3d object.
     * @param tolerance The tolerance range.
     * @return true if the poses are within the tolerance range, false otherwise.
     */
    private boolean isPoseWithinTolerance(Pose3d pose1, Pose3d pose2, double tolerance) {
        Translation3d translation1 = pose1.getTranslation();
        Translation3d translation2 = pose2.getTranslation();

        double deltaX = Math.abs(translation1.getX() - translation2.getX());
        double deltaY = Math.abs(translation1.getY() - translation2.getY());
        double deltaZ = Math.abs(translation1.getZ() - translation2.getZ());

        return deltaX <= tolerance && deltaY <= tolerance && deltaZ <= tolerance;
    }

    /**
     * Represents a pair of a DetectedObject and its associated confidence value.
     */
    public static class DetectedObjectPair {
        private DetectedObject object;
        private double confidence;

        /**
         * Constructs a new DetectedObjectPair with the specified DetectedObject and confidence value.
         *
         * @param object     The DetectedObject.
         * @param confidence The confidence value associated with the DetectedObject.
         */
        public DetectedObjectPair(DetectedObject object, double confidence) {
            this.object = object;
            this.confidence = confidence;
        }

        /**
         * Retrieves the DetectedObject.
         *
         * @return The DetectedObject.
         */
        public DetectedObject getObject() {
            return object;
        }

        /**
         * Sets the DetectedObject.
         *
         * @param object The DetectedObject to set.
         */
        public void setObject(DetectedObject object) {
            this.object = object;
        }

        /**
         * Retrieves the confidence value associated with the DetectedObject.
         *
         * @return The confidence value.
         */
        public double getConfidence() {
            return confidence;
        }

        /**
         * Sets the confidence value associated with the DetectedObject.
         *
         * @param confidence The confidence value to set.
         */
        public void setConfidence(double confidence) {
            this.confidence = confidence;
        }
        @Override
        public String toString() {
            return String.format("(X: %.2f, Y: %.2f, Z: %.2f, FieldAngle: %.2f) Class: %S, Conf: %.2f",
                    getObject().pose.getX(),
                    getObject().pose.getY(),
                    getObject().pose.getZ(),
                    getObject().getAngle(),
                    getObject().type,
                    getConfidence()
            );
        }
    }

    /**
     * Returns an array containing the string representation of each DetectedObjectPair in this list
     * in proper sequence (from first to last element).
     *
     * @return an array containing the string representation of each DetectedObjectPair in this list
     *         in proper sequence
     */
    @Override
    public String[] toArray() {
        String[] array = new String[size()];
        for (int i = 0; i < size(); i++) {
            DetectedObjectPair pair = get(i);
            array[i] = pair.toString();
        }
        return array;
    }
}