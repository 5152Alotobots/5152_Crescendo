package frc.robot.crescendo.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;


/**
 * Shoots at a given angle, velocity, and position
 */
public class Cmd_SubSys_Shooter_AmpShoot extends Command {
    SubSys_Shooter subSysShooter;
    boolean finished = false;

    Timer timer = new Timer();

    public Cmd_SubSys_Shooter_AmpShoot(
            SubSys_Shooter subSysShooter) {
        this.subSysShooter = subSysShooter;
        addRequirements(subSysShooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        subSysShooter.shootDumb(timer);
    }

    @Override
    public void end(boolean interrupted) {
        subSysShooter.stopAll();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
