package frc.robot.library.drivetrains.mecanum.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.library.driverstation.JoystickUtilities;
import frc.robot.library.drivetrains.mecanum.SubSys_MecanumDrive;
import java.util.function.DoubleSupplier;

public class Cmd_SubSys_MecanumDrive_JoystickDefault extends Command {
  /** Creates a new FalconTalonFXDriveTalonSR. */
  private final SubSys_MecanumDrive driveSubSys;

  private final DoubleSupplier fwdCmd;
  private final DoubleSupplier strCmd;
  private final DoubleSupplier rotCmd;

  /**
   * Cmd_SubSys_DriveTrain_JoysticDefault Joystick Drive Command
   *
   * @param driveSubSys
   * @param fwdCmd
   * @param strCmd
   * @param rotCmd
   */
  public Cmd_SubSys_MecanumDrive_JoystickDefault(
      SubSys_MecanumDrive mecanumDriveSubSys,
      DoubleSupplier fwdCmd,
      DoubleSupplier strCmd,
      DoubleSupplier rotCmd
    ) {

    this.driveSubSys = mecanumDriveSubSys;
    this.fwdCmd = fwdCmd;
    this.strCmd = strCmd;
    this.rotCmd = rotCmd;

    addRequirements(driveSubSys);
  }

  @Override
  public void execute() {

    driveSubSys.drive(
        JoystickUtilities.joyDeadBndSqrdScaled(fwdCmd.getAsDouble(), 0.05, 1),
        JoystickUtilities.joyDeadBndSqrdScaled(strCmd.getAsDouble(), 0.05, 1),
        JoystickUtilities.joyDeadBndScaled(rotCmd.getAsDouble(), 0.1, 1)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      driveSubSys.drive(0, 0, 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}