package frc.robot.crescendo.commands;

import static frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.PresetIntakePositions.INTAKE_PRESET_TRANSFER;
import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.PresentArmPositions.ARM_PRESET_TRANSFER;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.intake.IntakeDirection;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.util.ShooterIntakeDirection;

public class Cmd_TransferIntakeToShooter extends Command {
    /**
     * Creates a new FalconTalonFXDriveTalonSR.
     */
    private final SubSys_Shooter subSysShooter;
    private final SubSys_Intake subSysIntake;

    public Cmd_TransferIntakeToShooter(
            SubSys_Shooter subSysShooter,
            SubSys_Intake subSysIntake) {

        this.subSysShooter = subSysShooter;
        this.subSysIntake = subSysIntake;
        addRequirements(subSysShooter, subSysIntake);
    }

    @Override
    public void execute() {
        subSysIntake.setIntakeArmDegree(INTAKE_PRESET_TRANSFER); // Hold pos
        subSysShooter.setShooterArmDegree(ARM_PRESET_TRANSFER); // Hold position while transfering
        subSysShooter.setIntakeOutput(ShooterIntakeDirection.TRANSFER);
        subSysIntake.setIntakeDirection(IntakeDirection.TRANSFER);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subSysShooter.stopAll();
        subSysIntake.setIntakeDirection(IntakeDirection.NONE);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return subSysShooter.getIntakeOccupied();
    }
}
