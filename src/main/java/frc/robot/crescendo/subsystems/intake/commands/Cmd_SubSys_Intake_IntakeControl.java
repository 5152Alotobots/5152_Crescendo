package frc.robot.crescendo.subsystems.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.intake.IntakeDirection;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;

public class Cmd_SubSys_Intake_IntakeControl extends Command {
  /** Creates a new FalconTalonFXDriveTalonSR. */
  private final SubSys_Intake intakeSubSys;
  private final IntakeDirection intakeDirection;
  private final Timer timer = new Timer();

  public Cmd_SubSys_Intake_IntakeControl(SubSys_Intake intakeSubSys, IntakeDirection intakeDirection) {
    this.intakeSubSys = intakeSubSys;
    this.intakeDirection = intakeDirection;

    addRequirements(intakeSubSys);
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    intakeSubSys.setIntakeDirection(intakeDirection);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubSys.setIntakeDirection(IntakeDirection.NONE);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() > 1.0);
  }
}