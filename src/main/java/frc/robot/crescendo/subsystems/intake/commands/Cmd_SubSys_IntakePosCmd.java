// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;

public class Cmd_SubSys_IntakePosCmd extends Command {
  /** Creates a new Cmd_SubSys_IntakePosCmd. */
  private final SubSys_Intake intakeSubSys;
  private final double posCmd;
  private boolean atPos = false;

  /**
   * @deprecated Use {@link Cmd_SubSys_Intake_RotateToDegree}
   */
  @Deprecated
  public Cmd_SubSys_IntakePosCmd(SubSys_Intake intakeSubSys, double posCmd) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubSys = intakeSubSys;
    this.posCmd = posCmd;
    addRequirements(intakeSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    atPos = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    atPos = intakeSubSys.setIntakeArmPosCmd(posCmd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubSys.setIntakeArmSpd(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(atPos){
      return true;
    }else {
      return false;
    }
  }
}
