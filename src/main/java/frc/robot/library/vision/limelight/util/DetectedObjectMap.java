package frc.robot.library.vision.limelight.util;

import java.util.Objects;
import java.util.concurrent.ConcurrentSkipListMap;

import static frc.robot.library.vision.limelight.SubSys_Limelight_Constants.CONFIDENCE_DECAY;

/**
 * A thread-safe map implementation that extends ConcurrentSkipListMap to store DetectedObjects and their associated confidence values (double).
 * The map automatically decreases the confidence value of each entry by a fixed amount (CONFIDENCE_DECAY) at every update.
 * Entries with a confidence value of 0.0 or less are removed from the map.
 */
public class DetectedObjectMap extends ConcurrentSkipListMap<Integer, DetectedObjectMap.DetectedObjectConfidencePair> {


    /**
     * Constructs a new DetectedObjectMap instance.
     */
    public DetectedObjectMap() {
        super();
    }

    /**
     * Adds a new entry to the map with the specified key, DetectedObject, and confidence value.
     *
     * @param key        The key associated with the entry.
     * @param object     The DetectedObject to be stored in the map.
     * @param confidence The initial confidence value associated with the object (between 0.0 and 1.0).
     */
    public void put(int key, DetectedObject object, double confidence) {
        super.put(key, new DetectedObjectConfidencePair(object, confidence));
    }

    /**
     * Updates the map by decreasing the confidence value of each entry by CONFIDENCE_DECAY.
     * If the updated confidence value of an entry is 0.0 or less, the entry is removed from the map.
     */
    public void update() {
        for (Integer key : keySet()) {
            computeIfPresent(key, (k, pair) -> {
                double updatedConfidence = pair.confidence() - CONFIDENCE_DECAY;
                if (updatedConfidence <= 0.0) {
                    remove(k);
                    return null;
                } else {
                    return new DetectedObjectConfidencePair(pair.object(), updatedConfidence);
                }
            });
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
     * Private static nested class to store a DetectedObject and its associated confidence value (double).
     */
    public record DetectedObjectConfidencePair(DetectedObject object, double confidence) {
        /**
         * Retrieves the DetectedObject stored in the DetectedObjectConfidencePair.
         *
         * @return The DetectedObject.
         */
        @Override
        public DetectedObject object() {
            return object;
        }

        /**
         * Retrieves the confidence value associated with the DetectedObject.
         *
         * @return The confidence value.
         */
        @Override
        public double confidence() {
            return confidence;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            DetectedObjectConfidencePair that = (DetectedObjectConfidencePair) o;
            return Double.compare(that.confidence, confidence) == 0 &&
                    Objects.equals(object, that.object);
        }

        @Override
        public int hashCode() {
            return Objects.hash(object, confidence);
        }
    }
}