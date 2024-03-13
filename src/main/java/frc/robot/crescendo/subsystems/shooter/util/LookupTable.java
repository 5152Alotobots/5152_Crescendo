package frc.robot.crescendo.subsystems.shooter.util;

import java.util.HashMap;
import java.util.TreeMap;

@SuppressWarnings("ALL")
public class LookupTable<K extends Number, V> extends HashMap<K, V> {
    public V getValueForClosestKey(double target) {
        // Create a TreeMap from this HashMap
        TreeMap<K, V> sortedMap = new TreeMap<>(this);

        // Find the closest key
        K closestKey;
        if (sortedMap.containsKey((K) (Double) target)) {
            closestKey = (K) (Double) target;
        } else {
            K floorKey = sortedMap.floorKey((K) (Double) target);
            K ceilingKey = sortedMap.ceilingKey((K) (Double) target);

            double floorDistance = Double.MAX_VALUE;
            double ceilingDistance = Double.MAX_VALUE;

            if (floorKey != null) {
                floorDistance = Math.abs(floorKey.doubleValue() - target);
            }

            if (ceilingKey != null) {
                ceilingDistance = Math.abs(ceilingKey.doubleValue() - target);
            }

            closestKey = (floorDistance < ceilingDistance) ? floorKey : ceilingKey;
        }

        // Get the value for the closest key from this HashMap
        return this.get(closestKey);
    }
}