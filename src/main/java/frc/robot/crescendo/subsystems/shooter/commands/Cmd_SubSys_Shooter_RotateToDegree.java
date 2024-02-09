package frc.robot.crescendo.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;

public class Cmd_SubSys_Shooter_RotateToDegree extends Command {
    SubSys_Shooter subSysShooter;
    double degrees;

    Cmd_SubSys_Shooter_RotateToDegree(SubSys_Shooter subSysShooter, double degrees) {
        this.subSysShooter = subSysShooter;
        this.degrees = degrees;
    }

    @Override
    public void initialize() {
        subSysShooter.setShooterArmDegree(degrees);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return !subSysShooter.shooterArmMtrBusy();
    }
}
