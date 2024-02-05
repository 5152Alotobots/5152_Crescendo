package frc.robot.crescendo.subsystems.intake.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants;
import frc.robot.library.driverstation.JoystickUtilities;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Intake_Default extends Command {
  /** Creates a new FalconTalonFXDriveTalonSR. */
  private final SubSys_Intake intakeSubSys;
  private final double intakeSpeed; 
  private final DoubleSupplier deployerSpeed;

  public Cmd_SubSys_Intake_Default(SubSys_Intake intakeSubSys, double intakeSpeed, DoubleSupplier deployerSpeed) {
    this.intakeSubSys = intakeSubSys;
    this.intakeSpeed = intakeSpeed;
    this.deployerSpeed = deployerSpeed;

    addRequirements(intakeSubSys);
  }

  @Override
  public void execute() {
    if (deployerSpeed != null) {
      if (intakeSubSys.getForwardSensorValue()) {
        intakeSubSys.liftDeployer(JoystickUtilities.joyDeadBndScaled(-deployerSpeed.getAsDouble(), .5, .15));
        SmartDashboard.putBoolean("Force Lift", true);
        SmartDashboard.putBoolean("Force Lower", false);
      } else if (intakeSubSys.getReverseSensorValue()) {
        intakeSubSys.lowerDeployer(JoystickUtilities.joyDeadBndScaled(-deployerSpeed.getAsDouble(), 0.5, .15));
        SmartDashboard.putBoolean("Force Lift", false);
        SmartDashboard.putBoolean("Force Lower", true);
      } else {
        SmartDashboard.putBoolean("Force Lift", false);
        SmartDashboard.putBoolean("Force Lower", false);
        intakeSubSys.setDeployer(JoystickUtilities.joyDeadBndScaled(-deployerSpeed.getAsDouble(), 0.5, .15));
      }
    }

    if (!intakeSubSys.getRingSensorValue()) {
      intakeSubSys.setIntakeOutput(Math.max(0, intakeSpeed));
    } else {
      intakeSubSys.setIntakeOutput(intakeSpeed);
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      intakeSubSys.setIntakeOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
