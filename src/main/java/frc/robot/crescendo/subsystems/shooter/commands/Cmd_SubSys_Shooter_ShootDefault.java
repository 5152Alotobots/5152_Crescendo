package frc.robot.crescendo.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


/**
 * Shoots at a given angle, velocity, and position
 */
public class Cmd_SubSys_Shooter_ShootDefault extends Command {
    SubSys_Shooter subSysShooter;
    BooleanSupplier velocityWaitMode;
    DoubleSupplier shooterArmSpeed;
    Timer timer = new Timer();

    public Cmd_SubSys_Shooter_ShootDefault(
            SubSys_Shooter subSysShooter,
            BooleanSupplier velocityWaitMode,
            DoubleSupplier shooterArmSpeed) {
        this.subSysShooter = subSysShooter;
        this.velocityWaitMode = velocityWaitMode;
        this.shooterArmSpeed = shooterArmSpeed;

        addRequirements(subSysShooter);
    }

    @Override
    public void initialize() {
        if (!velocityWaitMode.getAsBoolean()) {
            timer.reset();
            timer.start();
        }
    }

    @Override
    public void execute() {
        subSysShooter.setShooterArmOutput(shooterArmSpeed.getAsDouble());

        if (velocityWaitMode.getAsBoolean()) {
            subSysShooter.shoot();
        } else {
            subSysShooter.shootDumb(timer);
        }
    }

    @Override
    public void end(boolean interrupted) {
        subSysShooter.stopAll();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return !subSysShooter.getIntakeOccupied(); // Should work, may want to test
    }
}
