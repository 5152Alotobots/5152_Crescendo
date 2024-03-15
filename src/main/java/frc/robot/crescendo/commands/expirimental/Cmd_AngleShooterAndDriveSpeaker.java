package frc.robot.crescendo.commands.expirimental;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.crescendo.subsystems.bling.SubSys_Bling;
import frc.robot.crescendo.subsystems.bling.commands.Cmd_SubSys_Bling_Shooting;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.commands.expirimental.Cmd_SubSys_Shooter_AutoAngleHoldThenShoot;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;
import frc.robot.library.drivetrains.swerve_ctre.commands.Cmd_SubSys_Drive_DriveWhileFacingSpeaker;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Cmd_AngleShooterAndDriveSpeaker extends ParallelCommandGroup
{
    public Cmd_AngleShooterAndDriveSpeaker(
            CommandSwerveDrivetrain drivetrain,
            SubSys_Bling subSysBling,
            SubSys_Shooter subSysShooter,
            DoubleSupplier velocityX,
            DoubleSupplier velocityY,
            BooleanSupplier releaseTrigger,
            DoubleSupplier shooterArmRotSpeed) {
        addCommands(
                new Cmd_SubSys_Bling_Shooting(subSysBling),
                new Cmd_SubSys_Drive_DriveWhileFacingSpeaker(drivetrain, velocityX, velocityY),
                new Cmd_SubSys_Shooter_AutoAngleHoldThenShoot(subSysShooter, drivetrain, releaseTrigger, shooterArmRotSpeed)
        );
    }
}
