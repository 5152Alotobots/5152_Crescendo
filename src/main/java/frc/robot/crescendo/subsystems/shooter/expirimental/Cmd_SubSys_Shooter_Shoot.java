package frc.robot.crescendo.subsystems.shooter.expirimental;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;

import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.AutoAim.LAUNCH_TOLERANCE;

/**
 * Shoots at a given angle, velocity, and position
 */
public class Cmd_SubSys_Shooter_Shoot extends Command {
    SubSys_Shooter subSysShooter;
    CommandSwerveDrivetrain subSysDrivetrain;
    Pose3d targetPose;
    boolean finished = false;

    public Cmd_SubSys_Shooter_Shoot(
            SubSys_Shooter subSysShooter,
            CommandSwerveDrivetrain subSysDrivetrain,
            Pose3d targetPose) {
        this.subSysShooter = subSysShooter;
        this.subSysDrivetrain = subSysDrivetrain;
        this.targetPose = targetPose;
        addRequirements(subSysShooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Pose2d calculatedLaunchPoint = AimModule.calculateLaunchPoint(
                subSysDrivetrain.getCurrentRobotChassisSpeeds().vxMetersPerSecond,
                subSysDrivetrain.getCurrentRobotChassisSpeeds().vyMetersPerSecond,
                targetPose.toPose2d(),
                subSysDrivetrain.getState().Pose);
        double calculatedLaunchAngle = AimModule.calculateLaunchAngle(targetPose);

        subSysShooter.setShooterArmDegree(calculatedLaunchAngle);
        if (Math.abs(calculatedLaunchPoint.relativeTo(subSysDrivetrain.getState().Pose).getX()) <= LAUNCH_TOLERANCE) {
            subSysShooter.shoot();
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        subSysShooter.stopAll();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
