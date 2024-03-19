package frc.robot.library.vision.limelight;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.crescendo.subsystems.bling.SubSys_Bling;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;

import static frc.robot.library.vision.limelight.SubSys_Limelight_Constants.*;

public class SubSys_Limelight extends SubsystemBase {
    private SubSys_Bling subSysBling;
    private CommandSwerveDrivetrain subSys_Drive;
    private String limelightName;
    private double horizontalOffset;
    private double verticalOffset;
    private double objectClass;
    private double distance;
    private DetectedObject note;
    private boolean objectDetected = false;
    private boolean limelightConnected;
    private LimelightLib.LimelightResults jsonDumpResults;

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
        limelightConnected =
                !NetworkTableInstance.getDefault()
                        .getTable(NN_LIMELIGHT)
                        .getEntry("json")
                        .getString("")
                        .isEmpty();

        if (OBJECT_DETECTION_ENABLED && limelightConnected) {
            objectDetected = LimelightLib.getTV(NN_LIMELIGHT);
            jsonDumpResults = LimelightLib.getLatestResults(NN_LIMELIGHT);

            if (objectDetected) {
                horizontalOffset = Math.toRadians(LimelightLib.getTX(NN_LIMELIGHT));
                verticalOffset = Math.toRadians(LimelightLib.getTY(NN_LIMELIGHT));
                double targetDist = targetDistanceMetersCamera(0, verticalOffset);
                note = new DetectedObject(horizontalOffset, verticalOffset, targetDist, DetectedObject.ObjectType.NOTE, LL_OFFSET);
            }
        }
    }
}
