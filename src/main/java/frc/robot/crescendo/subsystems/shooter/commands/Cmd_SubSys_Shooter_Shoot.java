package frc.robot.crescendo.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;

import java.util.function.BooleanSupplier;


/**
 * Shoots at a given angle, velocity, and position
 */
public class Cmd_SubSys_Shooter_Shoot extends Command {
    SubSys_Shooter subSysShooter;
    BooleanSupplier velocityWaitMode;

    Timer timer = new Timer();

    public Cmd_SubSys_Shooter_Shoot(
            SubSys_Shooter subSysShooter,
            BooleanSupplier velocityWaitMode) {
        this.subSysShooter = subSysShooter;
        this.velocityWaitMode = velocityWaitMode;

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
        return timer.hasElapsed(1.25);
        // return !subSysShooter.getIntakeOccupied(); // Should work, may want to test
    }
}
