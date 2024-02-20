package frc.robot.crescendo.subsystems.shooter.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AimModule {
    /**
     * Calculate the launch point given robot velocity, target coordinates, robot position, and rotation.
     *
     * @param xVel            X Velocity component of the robot (Shooter)
     * @param yVel            Y Velocity component of the robot (Left/Right)
     * @param targetX         Target X
     * @param targetY         Target Y
     * @param robotX          Robot X Position (Forward/Backwards facing speaker)
     * @param robotY          Robot Y Position (Left/Right facing speaker)
     * @param robotRotDegrees Robot Rotation in degrees (counterclockwise)
     * @return The launch point
     */
    private static double calculateLaunchPointExperimental(double xVel, double yVel, double targetX, double targetY,
                                                           double robotX, double robotY, double robotRotDegrees) {
        // Convert rotation angle to radians
        double radians = Math.toRadians(robotRotDegrees);

        // Translate target coordinates to the origin relative to the robot position so we can rotate around the origin
        double x1 = targetX - robotX;
        double y1 = targetY - robotY;

        // Apply rotation transformation
        double x2 = x1 * Math.cos(radians) - y1 * Math.sin(radians);
        double y2 = x1 * Math.sin(radians) + y1 * Math.cos(radians);

        // Translate back to the original coordinates after rotation
        targetX = x2 + robotX;
        targetY = y2 + robotY;

        // Calculate the slope of the line
        double m = yVel / xVel;

        // Calculate the launch point
        return targetX + (robotY - targetY) / m;
    }

    /**
     * Calculate the launch point given robot velocity, target coordinates, robot position, and rotation.
     *
     * @param xVel       X Velocity component of the robot (Shooter)
     * @param yVel       Y Velocity component of the robot (Left/Right)
     * @param targetPose Target X, Target Y, Target Rotation
     * @param robotPose  Robot X Position (Forward/Backwards facing speaker),
     *                   Robot Y Position (Left/Right facing speaker),
     *                   Robot Rotation in degrees (counterclockwise)
     * @return The launch point
     */
    private static double calculateLaunchPoint(double xVel, double yVel, Pose2d targetPose, Pose2d robotPose) {
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
        double m = yVel / xVel;

        // Calculate the launch point
        return targetX + (robotPose.getY() - targetY) / m;
    }

    public static void main(String[] args) {
        double calc = calculateLaunchPoint(6, 6, new Pose2d(0, 0, new Rotation2d()), new Pose2d(6, 3, new Rotation2d()));
        System.out.println("Calc: " + calc);
    }
}
