package frc.robot.crescendo.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;

public class Cmd_SubSys_Shooter_ArmPosCmd extends Command {
    private SubSys_Shooter subSysShooter;
    private double posCmd;
    private boolean atPos = false;

    /**
     * Rotates the shooter to a degree offset from flat 0 degrees
     */
    public Cmd_SubSys_Shooter_ArmPosCmd(SubSys_Shooter subSysShooter, double posCmd) {
        this.subSysShooter = subSysShooter;
        this.posCmd = posCmd;

    }

    @Override
    public void initialize() {
        subSysShooter.setShooterArmPosCmd(posCmd);
    }
  
    @Override
    public void execute() {
        atPos = subSysShooter.setShooterArmPosCmd(posCmd);
    }

    @Override
    public void end(boolean interrupted) {
        subSysShooter.setShooterArmDutyCycleCmd(0);
    }

    @Override
    public boolean isFinished() {
        return atPos;
    }
}
