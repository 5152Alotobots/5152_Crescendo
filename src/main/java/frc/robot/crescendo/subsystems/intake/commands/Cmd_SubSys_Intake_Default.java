package frc.robot.crescendo.subsystems.intake.commands;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;
import frc.robot.library.driverstation.JoystickUtilities;
import frc.robot.library.drivetrains.mecanum.SubSys_MecanumDrive;
import java.util.function.DoubleSupplier;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants;

public class Cmd_SubSys_Intake_Default extends Command {
  /** Creates a new FalconTalonFXDriveTalonSR. */
  private final SubSys_Intake intakeSubSys;

  
  private final DoubleSupplier percentOutput; 

  public Cmd_SubSys_Intake_Default(
      DoubleSupplier percentOutput,
      SubSys_Intake intakeSubSys
    ) {
    this.intakeSubSys = intakeSubSys;
    this.percentOutput = percentOutput;

    addRequirements(intakeSubSys);
  }

  @Override
  public void execute() {
    intakeSubSys.setIntakeOutput(SubSys_Intake_Constants.intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      intakeSubSys.setIntakeOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (intakeSubSys.getRingSensorValue()) {
      //TODO: Bling Code Here
      return true;
    } else {
      return false;
    }
  }
}
