package frc.robot.library.vision.limelight;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.crescendo.subsystems.bling.SubSys_Bling;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;
import frc.robot.library.vision.limelight.util.DetectedObject;
import frc.robot.library.vision.limelight.util.DetectedObjectMap;

import static frc.robot.library.vision.limelight.SubSys_Limelight_Constants.*;

public class SubSys_Limelight extends SubsystemBase {
    private SubSys_Bling subSysBling;
    private CommandSwerveDrivetrain subSys_Drive;
    private String limelightName;
    private DetectedObjectMap detectedObjectMap = new DetectedObjectMap();

    public SubSys_Limelight(String limelightName, SubSys_Bling subSysBling, CommandSwerveDrivetrain subSys_Drive) {
        this.limelightName = limelightName;
        this.subSysBling = subSysBling;
        this.subSys_Drive = subSys_Drive;
    }

    /**
     * Gets target distance from the camera
     *
     * @param targetHeight               distance from floor to center of target in meters
     * @param targetOffsetAngle_Vertical ty entry from limelight of target crosshair (in radians)
     * @return the distance to the target in meters
     */
    public double targetDistanceMetersCamera(
            double targetHeight,
            double targetOffsetAngle_Vertical) {
        double angleToGoalRadians = LL_OFFSET.getRotation().getY() + targetOffsetAngle_Vertical;
        return (targetHeight - LL_OFFSET.getZ()) / Math.tan(angleToGoalRadians);
    }


    @Override
    public void periodic() {
        // Returns false if JSON cannot be received
        boolean limelightConnected =
                !NetworkTableInstance.getDefault()
                        .getTable(NN_LIMELIGHT)
                        .getEntry("json")
                        .getString("")
                        .isEmpty();

        // If we are using getObject detection and we are connected
        if (OBJECT_DETECTION_ENABLED && limelightConnected) {
            // Get weather we have a detected getObject
            boolean objectDetected = LimelightLib.getTV(NN_LIMELIGHT);
            // If we do, get all JSON results
            if (objectDetected) {
                LimelightLib.LimelightResults latestResults = LimelightLib.getLatestResults(NN_LIMELIGHT);

                // Loop through every result in the array
                for (LimelightLib.LimelightTarget_Detector detection : latestResults.targetingResults.targets_Detector) {

                    // compute the offsets in radians (DetectedObject uses radians)
                    double horizontalOffset = Math.toRadians(detection.tx);
                    double verticalOffset = Math.toRadians(detection.ty);

                    // Compute the distance to the getObject
                    double targetDist = targetDistanceMetersCamera(0, verticalOffset);
                    DetectedObject note = new DetectedObject(horizontalOffset, verticalOffset, targetDist, DetectedObject.ObjectType.NOTE, LL_OFFSET);
                    detectedObjectMap.put(new DetectedObjectMap.DetectedObjectConfidencePair(note, detection.confidence));
                }
            }
        }
    }
}
