package frc.robot.crescendo.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.util.IntakeDirection;
import frc.robot.crescendo.subsystems.shooter.util.ShooterDirection;
import frc.robot.library.driverstation.JoystickUtilities;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Cmd_SubSys_Shooter_Default extends Command {
  /** Creates a new FalconTalonFXDriveTalonSR. */
  private final SubSys_Shooter subSysShooter;
  private final DoubleSupplier shooterArmSpeed;
  private final Supplier<ShooterDirection> shooterDirection;
  private final Supplier<IntakeDirection> intakeDirection;

  public Cmd_SubSys_Shooter_Default(
          SubSys_Shooter subSysShooter,
          DoubleSupplier shooterArmSpeed,
          Supplier<ShooterDirection> shooterDirection,
          Supplier<IntakeDirection> intakeDirection) {
    this.subSysShooter = subSysShooter;
    this.shooterArmSpeed = shooterArmSpeed;
    this.shooterDirection = shooterDirection;
    this.intakeDirection = intakeDirection;

    addRequirements(subSysShooter);
  }
  @Override
  public void execute() {
    // Arm Rotation
    subSysShooter.setShooterArmOutput(JoystickUtilities.joyDeadBndScaled(shooterArmSpeed.getAsDouble(), .5, .2));

    // If nothing is in the shooter, allow either way intake
    if (!subSysShooter.getIntakeOccupied()) subSysShooter.setIntakeOutput(intakeDirection.get());

    // If we want to shoot, spin the wheels and the intake at the same time
    if (shooterDirection.get().equals(ShooterDirection.OUT)) {
      subSysShooter.setShooterOutput(ShooterDirection.OUT);
      subSysShooter.setIntakeOutput(IntakeDirection.IN);
    } else {
      // Otherwise, allow reverse shooter or intake
      subSysShooter.setShooterOutput(shooterDirection.get());
      subSysShooter.setIntakeOutput(intakeDirection.get());
    }

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
