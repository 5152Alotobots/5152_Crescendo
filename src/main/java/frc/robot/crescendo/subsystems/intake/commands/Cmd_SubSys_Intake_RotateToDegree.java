package frc.robot.crescendo.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;

public class Cmd_SubSys_Intake_RotateToDegree extends Command {
    SubSys_Intake subSysIntake;
    double degrees;

    /**
     * Rotates the intake to a degree offset from (down position) 0 degrees
     */
    public Cmd_SubSys_Intake_RotateToDegree(SubSys_Intake subSysIntake, double degrees) {
        this.subSysIntake = subSysIntake;
        this.degrees = degrees;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        subSysIntake.setIntakeArmDegree(degrees);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return !subSysIntake.intakeArmMtrBusy();
    }
}
