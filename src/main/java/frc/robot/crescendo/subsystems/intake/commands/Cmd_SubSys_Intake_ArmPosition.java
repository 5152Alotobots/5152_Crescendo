package frc.robot.crescendo.subsystems.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;

import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Intake_ArmPosition extends Command {
    SubSys_Intake subSysIntake;
    double posCmd;
    boolean atPos = false;

    /**
     * Rotates the intake to a degree offset from (down position) 0 degrees
     */
    public Cmd_SubSys_Intake_ArmPosition(SubSys_Intake subSysIntake, double posCmd) {
        this.subSysIntake = subSysIntake;
        this.posCmd = posCmd;
    }

    @Override
    public void initialize() {
        subSysIntake.setIntakeArmMtrPosVolts(posCmd);
    }

    @Override
    public void execute() {
        atPos = subSysIntake.setIntakeArmMtrPosVolts(posCmd);
    }

    @Override
    public void end(boolean interrupted) {
        subSysIntake.setIntakeArmMtrDutyCycle(0);
    }

    @Override
    public boolean isFinished() {
        return atPos;
    }
}
