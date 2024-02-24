package frc.robot.crescendo.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.library.driverstation.JoystickUtilities;

import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Shooter_Default extends Command {
  /** Creates a new FalconTalonFXDriveTalonSR. */
  private final SubSys_Shooter subSysShooter;
  private final DoubleSupplier shooterArmSpeed;

  public Cmd_SubSys_Shooter_Default(
          SubSys_Shooter subSysShooter,
          DoubleSupplier shooterArmSpeed) {
    this.subSysShooter = subSysShooter;
    this.shooterArmSpeed = shooterArmSpeed;

    addRequirements(subSysShooter);
  }
  @Override
  public void execute() {
    // Arm Rotation
    subSysShooter.setShooterArmOutput(JoystickUtilities.joyDeadBndScaled(shooterArmSpeed.getAsDouble(), .5, 1));


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subSysShooter.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
