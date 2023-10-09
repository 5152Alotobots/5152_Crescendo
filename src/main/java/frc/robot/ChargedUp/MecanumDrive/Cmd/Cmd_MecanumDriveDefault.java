// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.MecanumDrive.Cmd;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ChargedUp.MecanumDrive.SubSys_MecanumDrive;
import java.util.function.DoubleSupplier;

public class Cmd_MecanumDriveDefault extends CommandBase {
  // Declare Variables
  private final SubSys_MecanumDrive m_MecanumDrive;
  private DoubleSupplier lefty;
  private DoubleSupplier leftx;
  private DoubleSupplier rightx;

  /**
   * Constructor
   *
   * @param MecanumDriveSubsystem
   * @param leftYJoy
   * @param leftXJoy
   * @param rightXJoy
   */
  public Cmd_MecanumDriveDefault(
      SubSys_MecanumDrive DriveSys,
      DoubleSupplier lefty,
      DoubleSupplier leftx,
      DoubleSupplier rightx) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_MecanumDrive = DriveSys;
    addRequirements(m_MecanumDrive);
    this.lefty = lefty;
    this.leftx = leftx;
    this.rightx = rightx;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_MecanumDrive.MecanumDrive(-lefty.getAsDouble(), -leftx.getAsDouble(), -rightx.getAsDouble());
    SmartDashboard.putNumber("Left Y (F/B)", -lefty.getAsDouble());
    SmartDashboard.putNumber("Left X (Strafe)", -leftx.getAsDouble());
    SmartDashboard.putNumber("Right X (Rotate)", -rightx.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
