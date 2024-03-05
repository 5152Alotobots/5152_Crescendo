package frc.robot.crescendo.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.util.ShooterDirection;
import frc.robot.crescendo.subsystems.shooter.util.ShooterIntakeDirection;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Cmd_SubSys_Shooter_Default extends Command {
  /** Creates a new FalconTalonFXDriveTalonSR. */
  private final SubSys_Shooter subSysShooter;
  private final DoubleSupplier shooterArmSpeed;
  private final Supplier<ShooterIntakeDirection> intakeDirection;
  private final Supplier<ShooterDirection> shooterDirection;

  public Cmd_SubSys_Shooter_Default(
          SubSys_Shooter subSysShooter,
          DoubleSupplier shooterArmSpeed,
          Supplier<ShooterIntakeDirection> intakeDirection,
          Supplier<ShooterDirection> shooterDirecton) {
    this.subSysShooter = subSysShooter;
    this.shooterArmSpeed = shooterArmSpeed;
    this.intakeDirection = intakeDirection;
    this.shooterDirection = shooterDirecton;

    addRequirements(subSysShooter);
  }
  @Override
  public void execute() {
    // Arm Rotation
    subSysShooter.setShooterArmOutput(shooterArmSpeed.getAsDouble());
    subSysShooter.setIntakeOutput(intakeDirection.get());
    subSysShooter.setShooterOutput(shooterDirection.get());

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
