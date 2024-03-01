package frc.robot.crescendo.subsystems.intake.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.intake.IntakeDirection;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.IntakeArm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Intake_SimpleDefault extends Command {
  /** Creates a new FalconTalonFXDriveTalonSR. */
  private final SubSys_Intake intakeSubSys;
  private final DoubleSupplier intakeArmSpeedAxis;
  private final BooleanSupplier intakeButton;
  private final BooleanSupplier outputButton;

  public Cmd_SubSys_Intake_SimpleDefault(SubSys_Intake intakeSubSys, DoubleSupplier intakeArmSpeedAxis, BooleanSupplier intakeButton, BooleanSupplier outputButton) {
    this.intakeSubSys = intakeSubSys;
    this.intakeArmSpeedAxis = intakeArmSpeedAxis;
    this.intakeButton = intakeButton;
    this.outputButton = outputButton;

    addRequirements(intakeSubSys);
  }

  @Override
  public void execute() {
    // Set Intake Arm Speed
    intakeSubSys.setIntakeArmDutyCycleCmd(intakeArmSpeedAxis.getAsDouble()*IntakeArm.MaxArmDutyCycle);

    // Set Intake Roller Speed
    if (intakeButton.getAsBoolean()) 
      intakeSubSys.intakeNote();
    else if (outputButton.getAsBoolean())
      intakeSubSys.ejectNote();
    else 
      intakeSubSys.setIntakeRollerDutyCycleCmd(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubSys.setIntakeArmDutyCycleCmd(0);
    intakeSubSys.setIntakeRollerDutyCycleCmd(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
