package frc.robot.crescendo.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.library.driverstation.JoystickUtilities;

import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Shooter_Default extends Command {
  /** Creates a new FalconTalonFXDriveTalonSR. */
  private final SubSys_Shooter shooterSubSys;
  private final double shooterSpeed; 
  private final DoubleSupplier shooterArmSpeed;
  private final double intakeSpeed;

  public Cmd_SubSys_Shooter_Default(SubSys_Shooter shooterSubSys, double shooterSpeed, double intakeSpeed, DoubleSupplier shooterArmSpeed) {
    this.shooterSubSys = shooterSubSys;
    this.shooterSpeed = shooterSpeed;
    this.shooterArmSpeed = shooterArmSpeed;
    this.intakeSpeed = intakeSpeed;

    addRequirements(shooterSubSys);
  }

  @Override
  public void execute() {
    if (shooterArmSpeed != null) {
      shooterSubSys.setShooterArmOutput(JoystickUtilities.joyDeadBndScaled(shooterArmSpeed.getAsDouble(), 0.5, .225));
    }
    shooterSubSys.setIntakeOutput(intakeSpeed);
    shooterSubSys.setShooterOutput(shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      shooterSubSys.setShooterArmOutput(0);
      shooterSubSys.setShooterOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
