package frc.robot.crescendo.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.ShooterArm;
import frc.robot.crescendo.subsystems.shooter.util.IntakeDirection;

public class Cmd_SubSys_Shooter_TransferFromIntake extends Command {
    /**
     * Creates a new FalconTalonFXDriveTalonSR.
     */
    private final SubSys_Shooter subSysShooter;
    
    public Cmd_SubSys_Shooter_TransferFromIntake(
            SubSys_Shooter subSysShooter) {

        this.subSysShooter = subSysShooter;
        addRequirements(subSysShooter);
    }

    @Override
    public void execute() {
        subSysShooter.transferNote();
        subSysShooter.setShooterArmPosCmd(ShooterArm.ShooterArmTransferPos);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subSysShooter.setShooterRollerDutyCycleCmd(0);
        subSysShooter.setShooterArmDutyCycleCmd(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return subSysShooter.getIntakeOccupied();
    }
}
