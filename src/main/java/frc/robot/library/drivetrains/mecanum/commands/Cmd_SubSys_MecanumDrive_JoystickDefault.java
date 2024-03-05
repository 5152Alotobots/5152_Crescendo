package frc.robot.library.drivetrains.mecanum.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.library.driverstation.JoystickUtilities;
import frc.robot.library.drivetrains.mecanum.SubSys_MecanumDrive;
import frc.robot.library.drivetrains.mecanum.SubSys_MecanumDrive_Constants;
import frc.robot.library.drivetrains.mecanum.SubSys_MecanumDrive_Constants.*;

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
    this.rotCmd = rotCmd;;

    addRequirements(mecanumDriveSubSys);
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("fwdCmd_Mec_Raw", fwdCmd.getAsDouble());
    //SmartDashboard.putNumber("strCmd_Mec_Raw", strCmd);
    //SmartDashboard.putNumber("rotCmd_Mec_Raw", rotCmd);

    /*
    driveSubSys.drive(
      AxisInverts.INVERT_FWD_AXIS ? 
        JoystickUtilities.joyDeadBndSqrd(-fwdCmd, SubSys_MecanumDrive_Constants.DEADBAND) : JoystickUtilities.joyDeadBndSqrd(fwdCmd, SubSys_MecanumDrive_Constants.DEADBAND),
      AxisInverts.INVERT_STR_AXIS ? 
        JoystickUtilities.joyDeadBndSqrd(-strCmd, SubSys_MecanumDrive_Constants.DEADBAND) : JoystickUtilities.joyDeadBndSqrd(strCmd, SubSys_MecanumDrive_Constants.DEADBAND),
      AxisInverts.INVERT_ROT_AXIS ? 
        JoystickUtilities.joyDeadBndSqrd(-rotCmd, SubSys_MecanumDrive_Constants.DEADBAND) : JoystickUtilities.joyDeadBndSqrd(rotCmd, SubSys_MecanumDrive_Constants.DEADBAND)
    );
    */
    driveSubSys.drive(-fwdCmd.getAsDouble(), strCmd.getAsDouble(), rotCmd.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    //if (!interrupted) {
      driveSubSys.drive(0, 0, 0);
    //}
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}