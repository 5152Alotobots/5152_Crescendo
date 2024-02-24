package frc.robot.crescendo.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.intake.IntakeDirection;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.util.ShooterIntakeDirection;

import java.util.function.BooleanSupplier;

import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.PresentArmPositions.PRESET_TRANSFER;

public class Cmd_SubSys_Shooter_Cycle extends Command {
    /**
     * Creates a new FalconTalonFXDriveTalonSR.
     */
    private final SubSys_Shooter subSysShooter;
    private final SubSys_Intake subSysIntake;
    private final BooleanSupplier intakeDownButton;
    private boolean endCommand = false;


    public Cmd_SubSys_Shooter_Cycle(
            SubSys_Shooter subSysShooter,
            SubSys_Intake subSysIntake,
            BooleanSupplier intakeDownButton) {

        this.subSysShooter = subSysShooter;
        this.subSysIntake = subSysIntake;
        this.intakeDownButton = intakeDownButton;
        addRequirements(subSysShooter, subSysIntake);
    }

    @Override
    public void execute() {
        // If intake button is enabled
        if (intakeDownButton.getAsBoolean()) {
            subSysIntake.setIntakeArmSpeed(1); // Down until limit switch
            subSysIntake.setIntakeDirection(IntakeDirection.IN);
            // If intake button is disabled and we have a note
        } else if (subSysIntake.getIntakeOccupied()) {
            // Start to move arm
            subSysShooter.setShooterArmDegree(PRESET_TRANSFER);
            subSysIntake.setIntakeArmSpeed(-1); // Move up til limit switch

            if (!subSysShooter.shooterArmMtrBusy() && subSysIntake.atUpperLimit()) {
                // Transfer
                subSysShooter.setIntakeOutput(ShooterIntakeDirection.TRANSFER);
                subSysIntake.setIntakeDirection(IntakeDirection.TRANSFER);
                // End if shooter occupied
                endCommand = subSysIntake.getIntakeOccupied();
            }
        } else {
            // If no note, reset to normal position and end the command
            subSysIntake.setIntakeArmSpeed(-1);
            subSysIntake.setIntakeDirection(IntakeDirection.NONE);
            endCommand = true;
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subSysShooter.stopAll();
        subSysIntake.setIntakeArmSpeed(0);
        subSysIntake.setIntakeDirection(IntakeDirection.NONE);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return endCommand;
    }
}
