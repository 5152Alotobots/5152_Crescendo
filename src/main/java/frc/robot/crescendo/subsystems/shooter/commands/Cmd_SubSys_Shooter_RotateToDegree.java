package frc.robot.crescendo.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;

import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Shooter_RotateToDegree extends Command {
    SubSys_Shooter subSysShooter;
    DoubleSupplier degrees;
    Timer timer = new Timer();

    /**
     * @param degrees The degree to rotate to 
     * Rotates the shooter to a degree offset from flat 0 degrees
     */
    public Cmd_SubSys_Shooter_RotateToDegree(SubSys_Shooter subSysShooter, DoubleSupplier degrees) {
        this.subSysShooter = subSysShooter;
        this.degrees = degrees;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }
  
    @Override
    public void execute() {
        subSysShooter.setShooterArmDegree(degrees.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        subSysShooter.setShooterArmOutput(0);
    }

    @Override
    public boolean isFinished() {
        return subSysShooter.shooterArmMtrAtSetpoint() && timer.get() > 0.8; // account for delay
    }
}
