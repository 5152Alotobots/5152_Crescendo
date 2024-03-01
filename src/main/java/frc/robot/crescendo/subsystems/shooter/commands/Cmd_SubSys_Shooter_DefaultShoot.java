package frc.robot.crescendo.subsystems.shooter.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.ShooterArm;


/**
 * Shoots at a given angle, velocity, and position
 */
public class Cmd_SubSys_Shooter_DefaultShoot extends Command {
    private SubSys_Shooter subSysShooter;
    private final DoubleSupplier shooterArmCmd;

    boolean finished = false;

    Timer timer = new Timer();

    public Cmd_SubSys_Shooter_DefaultShoot(
            SubSys_Shooter subSysShooter,
            DoubleSupplier shooterArmCmd) {

        this.subSysShooter = subSysShooter;
        this.shooterArmCmd = shooterArmCmd;
        addRequirements(subSysShooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        subSysShooter.setShooterArmPosCmd(shooterArmCmd.getAsDouble());
        subSysShooter.shootSpeakerShooterWheels();
    }

    @Override
    public void execute() {
        subSysShooter.setShooterArmPosCmd(shooterArmCmd.getAsDouble());
        subSysShooter.shootSpeakerShooterWheels();
        
        if(timer.hasElapsed(2.0)){
            subSysShooter.shootNoteRollers();
        }
    }

    @Override
    public void end(boolean interrupted) {
        subSysShooter.setShooterRollerDutyCycleCmd(0);
        subSysShooter.setShooterWheelsDutyCycleCmd(0);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(3.5);
    }
}
