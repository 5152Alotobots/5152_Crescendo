package frc.robot.crescendo.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.intake.IntakePosition;
import frc.robot.crescendo.subsystems.intake.IntakeSpeed;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;

import java.util.function.Supplier;

public class Cmd_SubSys_Intake_Default extends Command {
  /** Creates a new FalconTalonFXDriveTalonSR. */
  private final SubSys_Intake intakeSubSys;
  private final Supplier<IntakeSpeed> intakeSpeed;
  private final Supplier<IntakePosition> intakePosition;

  public Cmd_SubSys_Intake_Default(SubSys_Intake intakeSubSys, Supplier<IntakePosition> intakePosition, Supplier<IntakeSpeed> intakeSpeed) {
    this.intakeSubSys = intakeSubSys;
    this.intakeSpeed = intakeSpeed;
    this.intakePosition = intakePosition;

    addRequirements(intakeSubSys);
  }

  @Override
  public void execute() {
    intakeSubSys.setIntakeSpeed(intakeSpeed.get());
    intakeSubSys.setIntakeArmPosition(intakePosition.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
