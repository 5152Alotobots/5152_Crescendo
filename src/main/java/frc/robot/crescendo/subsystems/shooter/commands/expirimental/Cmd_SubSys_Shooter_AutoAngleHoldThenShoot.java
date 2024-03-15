package frc.robot.crescendo.subsystems.shooter.commands.expirimental;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.expirimental.AimModule;
import frc.robot.crescendo.subsystems.shooter.util.ShooterDirection;
import frc.robot.crescendo.subsystems.shooter.util.ShooterIntakeDirection;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.AutoAim.SHOOTER_WAIT_AFTER_SHOOT;

public class Cmd_SubSys_Shooter_AutoAngleHoldThenShoot extends Command {
    SubSys_Shooter subSysShooter;
    CommandSwerveDrivetrain drivetrain;
    BooleanSupplier releaseTrigger;
    DoubleSupplier shooterArmSpeed;
    boolean manualOverride = false;

    Timer timer = new Timer();

    public Cmd_SubSys_Shooter_AutoAngleHoldThenShoot(
            SubSys_Shooter subSysShooter,
            CommandSwerveDrivetrain drivetrain,
            BooleanSupplier releaseTrigger,
            DoubleSupplier shooterArmSpeed) {
        this.subSysShooter = subSysShooter;
        this.drivetrain = drivetrain;
        this.releaseTrigger = releaseTrigger;
        this.shooterArmSpeed = shooterArmSpeed;

        addRequirements(subSysShooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        manualOverride = false;
    }

    @Override
    public void execute() {
        if (shooterArmSpeed.getAsDouble() > 0.05) {
            manualOverride = true;
        }
        if (manualOverride) {
            subSysShooter.setShooterArmOutput(shooterArmSpeed.getAsDouble());
        } else {
            subSysShooter.setShooterArmDegree(AimModule.calculateLaunchAngle(drivetrain.getState().Pose));
        }
        subSysShooter.setShooterOutput(ShooterDirection.OUT);
        if (releaseTrigger.getAsBoolean()) {
            subSysShooter.setIntakeOutput(ShooterIntakeDirection.SHOOT);
            timer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        subSysShooter.stopAll();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return !subSysShooter.getIntakeOccupied() && timer.get() > SHOOTER_WAIT_AFTER_SHOOT;
    }
}
