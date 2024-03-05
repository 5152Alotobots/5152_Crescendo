package frc.robot.crescendo.subsystems.intake.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.intake.IntakeDirection;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Intake_Default extends Command {
  /** Creates a new FalconTalonFXDriveTalonSR. */
  private final SubSys_Intake intakeSubSys;
  private final DoubleSupplier intakeArmSpeedAxis;
  private final BooleanSupplier intakeButton;
  private final BooleanSupplier outputButton;

  public Cmd_SubSys_Intake_Default(SubSys_Intake intakeSubSys, DoubleSupplier intakeArmSpeedAxis, BooleanSupplier intakeButton, BooleanSupplier outputButton) {
    this.intakeSubSys = intakeSubSys;
    this.intakeArmSpeedAxis = intakeArmSpeedAxis;
    this.intakeButton = intakeButton;
    this.outputButton = outputButton;

    addRequirements(intakeSubSys);
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Intake/Input Axis", intakeArmSpeedAxis.getAsDouble());
    intakeSubSys.setIntakeArmSpeedWithLimits(intakeArmSpeedAxis.getAsDouble());
    if (intakeButton.getAsBoolean()) 
      intakeSubSys.setIntakeDirection(IntakeDirection.IN);
    else if (outputButton.getAsBoolean())
      intakeSubSys.setIntakeDirection(IntakeDirection.TRANSFER);
    else 
      intakeSubSys.setIntakeDirection(IntakeDirection.NONE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubSys.setIntakeArmSpeed(0);
    intakeSubSys.setIntakeDirection(IntakeDirection.NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
