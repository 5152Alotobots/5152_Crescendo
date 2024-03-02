package frc.robot.crescendo.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.util.ShooterDirection;
import frc.robot.crescendo.subsystems.shooter.util.ShooterIntakeDirection;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.AutoAim.SHOOTER_WAIT_AFTER_SHOOT;

public class Cmd_SubSys_Shooter_HoldThenShoot extends Command {
    SubSys_Shooter subSysShooter;
    BooleanSupplier releaseTrigger;
    DoubleSupplier shooterArmSpeed;

    Timer timer = new Timer();

    public Cmd_SubSys_Shooter_HoldThenShoot(
            SubSys_Shooter subSysShooter,
            BooleanSupplier releaseTrigger,
            DoubleSupplier shooterArmSpeed) {
        this.subSysShooter = subSysShooter;
        this.releaseTrigger = releaseTrigger;
        this.shooterArmSpeed = shooterArmSpeed;

        addRequirements(subSysShooter);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        subSysShooter.setShooterArmOutput(shooterArmSpeed.getAsDouble());
        subSysShooter.setShooterOutput(ShooterDirection.OUT);
        if (releaseTrigger.getAsBoolean()) {
            subSysShooter.setIntakeOutput(ShooterIntakeDirection.IN);
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
