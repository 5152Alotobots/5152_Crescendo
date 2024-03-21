package frc.robot.library.vision.limelight.util;

import edu.wpi.first.math.geometry.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ConcurrentSkipListMap;

import static frc.robot.library.vision.limelight.SubSys_Limelight_Constants.*;

/**
 * A thread-safe map implementation that extends ConcurrentSkipListMap to store DetectedObjects and their associated getConfidence values (double).
 * The map automatically decreases the getConfidence value of each entry by a fixed amount (CONFIDENCE_DECAY) at every update.
 * Entries with a getConfidence value of 0.0 or less are removed from the map.
 */
public class DetectedObjectMap extends ConcurrentSkipListMap<Integer, DetectedObjectMap.DetectedObjectConfidencePair> {


    /**
     * Constructs a new DetectedObjectMap instance.
     */
    public DetectedObjectMap() {
        super();
    }

    /**
     * Adds a new entry to the map with the specified key, DetectedObject, and getConfidence value.
     *
     * @param key        The key associated with the entry.
     * @param object     The DetectedObject to be stored in the map.
     * @param confidence The initial getConfidence value associated with the getObject (between 0.0 and 1.0).
     */
    public void put(int key, DetectedObject object, double confidence) {
        super.put(key, new DetectedObjectConfidencePair(object, confidence));
    }

    /**
     * Adds a new entry to the map with the specified DetectedObject and getConfidence value.
     * If the new DetectedObject's pose is within a certain range (POSE_TOLERANCE) of an existing pose,
     * the existing DetectedObject is updated instead of adding a new one.
     *
     * @param pair The DetectedObjectConfidencePair to be added or updated in the map.
     */
    public void put(DetectedObjectConfidencePair pair) {
        for (Entry<Integer, DetectedObjectConfidencePair> entry : entrySet()) {
            DetectedObject existingObject = entry.getValue().getObject();
            DetectedObject newObject = pair.getObject();

            if (isPoseWithinTolerance(existingObject.pose, newObject.pose, POSE_TOLERANCE)) {
                // Update the existing DetectedObject with the new confidence and return
                put(entry.getKey(), new DetectedObjectConfidencePair(newObject, pair.getConfidence()));
                return;
            }
        }

        // If no existing DetectedObject within the pose tolerance is found, add a new entry
        int nextKey = size();
        super.put(nextKey, pair);
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
     * Updates the map by decreasing the getConfidence value of each entry by CONFIDENCE_DECAY.
     * If the updated getConfidence value of an entry is 0.0 or less, the entry is removed from the map.
     */
    public void update() {
        for (Integer key : keySet()) {
            computeIfPresent(key, (k, pair) -> {
                double updatedConfidence = pair.getConfidence() - CONFIDENCE_DECAY;
                if (updatedConfidence <= 0.0) {
                    remove(k);
                    return null;
                } else {
                    return new DetectedObjectConfidencePair(pair.getObject(), updatedConfidence);
                }
            });
        }
    }

    /**
     * Sorts the map based on the getConfidence values, with the highest getConfidence value at index 0 and decreasing as the index increases.
     */
    public void sortByConfidence() {
        List<Entry<Integer, DetectedObjectConfidencePair>> entryList = new ArrayList<>(entrySet());
        entryList.sort(Map.Entry.comparingByValue((a, b) -> Double.compare(b.getConfidence(), a.getConfidence())));

        clear();
        for (int i = 0; i < entryList.size(); i++) {
            put(i, entryList.get(i).getValue());
        }
    }

    /**
     * Sorts the map based on the distance from the given Pose2d to the Pose3d of each DetectedObject.
     * The DetectedObject with the closest Pose3d to the given Pose2d will be at index 0, and the distance increases as the index increases.
     *
     * @param pose The Pose2d to calculate distances from.
     */
    public void sortByPose(Pose2d pose) {
        List<Map.Entry<Integer, DetectedObjectConfidencePair>> entryList = new ArrayList<>(entrySet());
        entryList.sort(Map.Entry.comparingByValue((a, b) -> {
            Translation2d translationA = new Translation2d(a.getObject.pose.getX(), a.getObject.pose.getY());
            Translation2d translationB = new Translation2d(b.getObject.pose.getX(), b.getObject.pose.getY());
            double distanceA = pose.getTranslation().getDistance(translationA);
            double distanceB = pose.getTranslation().getDistance(translationB);
            return Double.compare(distanceA, distanceB);
        }));

        clear();
        for (int i = 0; i < entryList.size(); i++) {
            put(i, entryList.get(i).getValue());
        }
    }

    /**
     * Checks if an entry with the specified key is present in the map.
     *
     * @param key The key to check for.
     * @return true if an entry with the specified key is present, false otherwise.
     */
    public boolean isPresent(int key) {
        return containsKey(key);
    }

    /**
     * Private static nested class to store a DetectedObject and its associated getConfidence value (double).
     */
    public record DetectedObjectConfidencePair(DetectedObject getObject, double getConfidence) {
        /**
         * Retrieves the DetectedObject stored in the DetectedObjectConfidencePair.
         *
         * @return The DetectedObject.
         */
        public DetectedObject getObject() {
            return getObject;
        }

        /**
         * Retrieves the getConfidence value associated with the DetectedObject.
         *
         * @return The getConfidence value.
         */
        public double getConfidence() {
            return getConfidence;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            DetectedObjectConfidencePair that = (DetectedObjectConfidencePair) o;
            return Double.compare(that.getConfidence, getConfidence) == 0 &&
                    Objects.equals(getObject, that.getObject);
        }

        @Override
        public int hashCode() {
            return Objects.hash(getObject, getConfidence);
        }
    }
}