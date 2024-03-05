package frc.robot.crescendo.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;

import java.util.function.DoubleSupplier;


/**
 * Shoots at a given angle, velocity, and position
 */
public class Cmd_SubSys_Shooter_ShootAtSpeed extends Command {
    SubSys_Shooter subSysShooter;
    DoubleSupplier speed;

    Timer timer = new Timer();

    public Cmd_SubSys_Shooter_ShootAtSpeed(
            SubSys_Shooter subSysShooter,
            DoubleSupplier speed) {
        this.subSysShooter = subSysShooter;
        this.speed = speed;

        addRequirements(subSysShooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        subSysShooter.shootDumbAtSpeed(timer, speed.getAsDouble());
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
