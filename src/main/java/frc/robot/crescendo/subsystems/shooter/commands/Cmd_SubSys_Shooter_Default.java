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
  private final SubSys_Shooter shooterSubSys;
  private final DoubleSupplier shooterArmSpeed;
  private final Supplier<ShooterDirection> shooterDirection;
  private final Supplier<IntakeDirection> intakeDirection;

  public Cmd_SubSys_Shooter_Default(
          SubSys_Shooter shooterSubSys,
          DoubleSupplier shooterArmSpeed,
          Supplier<ShooterDirection> shooterDirection,
          Supplier<IntakeDirection> intakeDirection) {
    this.shooterSubSys = shooterSubSys;
    this.shooterArmSpeed = shooterArmSpeed;
    this.shooterDirection = shooterDirection;
    this.intakeDirection = intakeDirection;

    addRequirements(shooterSubSys);
  }
  @Override
  public void execute() {
    // Arm Rotation
    shooterSubSys.setShooterArmOutput(JoystickUtilities.joyDeadBndScaled(shooterArmSpeed.getAsDouble(), .5, .2));
    if (!shooterSubSys.getIntakeOccupied()) {
      shooterSubSys.setIntakeOutput(intakeDirection.get());
    }
    if (shooterDirection.get().equals(ShooterDirection.OUT)) {
      shooterSubSys.setShooterOutput(ShooterDirection.OUT);
        shooterSubSys.setIntakeOutput(IntakeDirection.IN);
    } else {
      
      shooterSubSys.setShooterOutput(shooterDirection.get());
      shooterSubSys.setIntakeOutput(intakeDirection.get());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubSys.setShooterArmOutput(0);
    shooterSubSys.setIntakeOutput(IntakeDirection.OFF);
    shooterSubSys.setShooterOutput(ShooterDirection.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
