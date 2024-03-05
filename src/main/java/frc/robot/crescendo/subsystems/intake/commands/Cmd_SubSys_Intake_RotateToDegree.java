package frc.robot.crescendo.subsystems.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;

import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Intake_RotateToDegree extends Command {
    SubSys_Intake subSysIntake;
    DoubleSupplier degrees;
    Timer timer = new Timer();

    /**
     * Rotates the intake to a degree offset from (down position) 0 degrees
     */
    public Cmd_SubSys_Intake_RotateToDegree(SubSys_Intake subSysIntake, DoubleSupplier degrees) {
        this.subSysIntake = subSysIntake;
        this.degrees = degrees;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        subSysIntake.setIntakeArmDegree(degrees.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        subSysIntake.setIntakeArmSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return subSysIntake.intakeArmMtrAtSetpoint() && timer.get() > 0.8; // account for delay
    }
}
