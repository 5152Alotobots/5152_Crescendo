package frc.robot.library.vision.limelight;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.crescendo.subsystems.bling.SubSys_Bling;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;
import frc.robot.library.vision.limelight.util.DetectedObject;
import frc.robot.library.vision.limelight.util.DetectedObjectList;

import java.util.Map;

import static frc.robot.library.vision.limelight.SubSys_Limelight_Constants.*;

public class SubSys_Limelight extends SubsystemBase {
    private SubSys_Bling subSysBling;
    private CommandSwerveDrivetrain subSys_Drive;
    private DetectedObjectList detectedObjectList = new DetectedObjectList();
    private ShuffleboardLayout detectedObjectsShuffleboard;
    public SubSys_Limelight(SubSys_Bling subSysBling, CommandSwerveDrivetrain subSys_Drive) {
        this.subSysBling = subSysBling;
        this.subSys_Drive = subSys_Drive;
        DetectedObject.setDrive(subSys_Drive);
        detectedObjectsShuffleboard = Shuffleboard
                .getTab("Limelight")
                .getLayout("Detected Objects", BuiltInLayouts.kGrid)
                .withProperties(Map.of("Number of columns", 1, "Number of rows", 5));
        detectedObjectsShuffleboard.addStringArray("Detected Objects", () -> detectedObjectList.toArray());
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

            // Update decay
            detectedObjectList.update();
            if (objectDetected) {
                LimelightLib.LimelightResults latestResults = LimelightLib.getLatestResults(NN_LIMELIGHT);

                // Loop through every result in the array
                for (LimelightLib.LimelightTarget_Detector detection : latestResults.targetingResults.targets_Detector) {

                    // compute the offsets in radians (DetectedObject uses radians)
                    double horizontalOffset = -Math.toRadians(detection.tx); // MAKE CCW POS
                    double verticalOffset = Math.toRadians(detection.ty);

                    // Compute the distance to the getObject
                    double targetDist = targetDistanceMetersCamera(0, verticalOffset);
                    DetectedObject note = new DetectedObject(horizontalOffset, verticalOffset, targetDist, DetectedObject.ObjectType.NOTE, LL_OFFSET);
                    detectedObjectList.add(new DetectedObjectList.DetectedObjectPair(note, detection.confidence));
                }
            }
        }
    }
}
