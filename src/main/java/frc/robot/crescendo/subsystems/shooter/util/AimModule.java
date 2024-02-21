package frc.robot.crescendo.subsystems.shooter.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.MaxSpeeds.MAX_SHOOTER_SPPED_MPS;

public class AimModule {
    /**
     * Calculate the launch point given robot velocity, target coordinates, robot position, and rotation.
     *
     * @param xRobotVel  X Velocity component of the robot (Forward/Backwards facing speaker)
     * @param yRobotVel  Y Velocity component of the robot (Left/Right facing speaker)
     * @param targetPose Target X, Target Y, Target Rotation
     * @param robotPose  Robot X Position (Forward/Backwards facing speaker),
     *                   Robot Y Position (Left/Right facing speaker),
     *                   Robot Rotation in degrees (counterclockwise)
     * @return The launch point (Pose2d)
     */
    public static Pose2d calculateLaunchPoint(double xRobotVel, double yRobotVel, Pose2d targetPose, Pose2d robotPose) {
        // Convert rotation angle to radians
        double radians = targetPose.getRotation().minus(robotPose.getRotation()).getRadians();
        // Translate target coordinates to the origin relative to the robot position, so we can rotate around the origin
        double x1 = targetPose.getX() - robotPose.getX();
        double y1 = targetPose.getY() - robotPose.getY();

        // Apply rotation transformation
        double x2 = x1 * Math.cos(radians) - y1 * Math.sin(radians);
        double y2 = x1 * Math.sin(radians) + y1 * Math.cos(radians);

        // Translate back to the original coordinates after rotation
        double targetX = x2 + robotPose.getX();
        double targetY = y2 + robotPose.getY();

        // Calculate the slope of the line
        double m = (yRobotVel + MAX_SHOOTER_SPPED_MPS) / xRobotVel;

        // Calculate the launch point
        return new Pose2d((targetX + (robotPose.getY() - targetY) / m), robotPose.getY(), robotPose.getRotation());
    }

    /**
     * @return The launch angle (degrees)
     */
    public static double calculateLaunchAngle(Pose3d targetPose) {
        return 0; // 0 degrees offset from flat
    }

}
