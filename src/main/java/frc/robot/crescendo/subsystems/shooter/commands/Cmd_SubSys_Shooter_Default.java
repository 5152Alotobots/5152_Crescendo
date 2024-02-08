package frc.robot.crescendo.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.IntakeSpeed;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Cmd_SubSys_Shooter_Default extends Command {
  /** Creates a new FalconTalonFXDriveTalonSR. */
  private final SubSys_Shooter shooterSubSys;
  private final DoubleSupplier shooterSpeed;
  private final DoubleSupplier shooterArmSpeed;
  private final Supplier<IntakeSpeed> intakeSpeed;

  public Cmd_SubSys_Shooter_Default(
          SubSys_Shooter shooterSubSys,
          Supplier<IntakeSpeed> intakeSpeed,
          DoubleSupplier shooterSpeed,
          DoubleSupplier shooterArmSpeed) {
    this.shooterSubSys = shooterSubSys;
    this.shooterSpeed = shooterSpeed;
    this.shooterArmSpeed = shooterArmSpeed;
    this.intakeSpeed = intakeSpeed;

    addRequirements(shooterSubSys);
  }

  @Override
  public void execute() {
    shooterSubSys.setIntakeOutput(intakeSpeed.get());
    shooterSubSys.setShooterArmOutput(shooterArmSpeed.getAsDouble());
    shooterSubSys.setShooterOutput(shooterSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubSys.setIntakeOutput(IntakeSpeed.OFF);
    shooterSubSys.setShooterArmOutput(0);
    shooterSubSys.setShooterOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
