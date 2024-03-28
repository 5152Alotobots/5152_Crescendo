package frc.robot.crescendo.subsystems.shooter.expirimental;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.bling.SubSys_Bling;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;

import static frc.robot.crescendo.subsystems.bling.SubSys_Bling_Constants.Animations.SHOOTING_ANIMATION;
import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.AutoAim.LAUNCH_TOLERANCE;
import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.SPEAKER_POSE_BLUE;
import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.SPEAKER_POSE_RED;

/**
 * Shoots at a given angle, velocity, and position
 */
public class Cmd_SubSys_Shooter_AutoShoot extends Command {
    SubSys_Shooter subSysShooter;
    CommandSwerveDrivetrain subSysDrivetrain;
    SubSys_Bling subSysBling;
    Pose2d targetPose;
    boolean finished = false;

    public Cmd_SubSys_Shooter_AutoShoot(
            SubSys_Shooter subSysShooter,
            SubSys_Bling subSysBling,
            CommandSwerveDrivetrain subSysDrivetrain) {
        this.subSysShooter = subSysShooter;
        this.subSysDrivetrain = subSysDrivetrain;
        this.subSysBling = subSysBling;
        addRequirements(subSysShooter, subSysBling);
    }

    @Override
    public void initialize() {
        subSysBling.runAnimation(SHOOTING_ANIMATION);
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
                targetPose = SPEAKER_POSE_RED;
            } else {
                targetPose = SPEAKER_POSE_BLUE;
            }
        } else {
            finished = true; // If no alliance don't auto shoot
        }
    }

    @Override
    public void execute() {
        subSysBling.update();
        Pose2d calculatedLaunchPoint = AimModule.calculateLaunchPoint(
                subSysDrivetrain.getCurrentRobotChassisSpeeds().vxMetersPerSecond,
                subSysDrivetrain.getCurrentRobotChassisSpeeds().vyMetersPerSecond,
                targetPose,
                subSysDrivetrain.getState().Pose);
        double calculatedLaunchAngle = AimModule.calculateLaunchAngle(subSysDrivetrain.getState().Pose);
        SmartDashboard.putNumber("Calculate Launch Angle", calculatedLaunchAngle);
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
