package frc.robot.crescendo.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.util.IntakeDirection;

public class Cmd_SubSys_Shooter_Transfer extends Command {
    /**
     * Creates a new FalconTalonFXDriveTalonSR.
     */
    private final SubSys_Shooter subSysShooter;
    private final SubSys_Intake subSysIntake;

    public Cmd_SubSys_Shooter_Transfer(
            SubSys_Shooter subSysShooter,
            SubSys_Intake subSysIntake) {

        this.subSysShooter = subSysShooter;
        this.subSysIntake = subSysIntake;
        addRequirements(subSysShooter, subSysIntake);
    }

    @Override
    public void execute() {
        subSysShooter.setIntakeOutput(IntakeDirection.TRANSFER);
        subSysIntake.setIntakeDirection(frc.robot.crescendo.subsystems.intake.IntakeDirection.TRANSFER);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subSysShooter.stopAll();
        subSysIntake.setIntakeDirection(frc.robot.crescendo.subsystems.intake.IntakeDirection.NONE);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return subSysShooter.getIntakeOccupied();
    }
}