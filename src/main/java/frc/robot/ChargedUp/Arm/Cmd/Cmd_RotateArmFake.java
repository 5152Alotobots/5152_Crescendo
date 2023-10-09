// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Arm.Cmd;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
// import frc.robot.ChargedUp.DriverStation.SubSys_DriverStation;
// import java.util.function.DoubleSupplier;
import java.util.function.DoubleSupplier;

public class Cmd_RotateArmFake extends CommandBase {
  /** Creates a new Cmd_RotateArm. */
  private final SubSys_Arm ArmSubsys;

  private final DoubleSupplier Axis;

  public Cmd_RotateArmFake(SubSys_Arm ArmSubsys, DoubleSupplier Axis) {
    this.ArmSubsys = ArmSubsys;
    this.Axis = Axis;
    addRequirements(ArmSubsys);
  }

  public double FalseEncoderValue;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    FalseEncoderValue = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Axis for Arm Rot", Axis.getAsDouble());
    SmartDashboard.putNumber("False Encoder Value", FalseEncoderValue);
    SmartDashboard.putNumber(
        "Length Of Arm", ArmSubsys.getLengthOfArmFromBase(FalseEncoderValue, 3));
    SmartDashboard.putNumber(
        "Height Of Arm", ArmSubsys.getHeightOfArmFromBase(FalseEncoderValue, 3));
    if (Axis.getAsDouble() > .05 || Axis.getAsDouble() < -.05) {
      FalseEncoderValue = FalseEncoderValue + Axis.getAsDouble();
    }
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
