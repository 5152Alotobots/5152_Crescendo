// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.climber.SubSys_Climber;

public class climberSetVoltUp extends Command {
  /** Creates a new climberSetVolt. */
  private final SubSys_Climber m_climberSubSys;

  public climberSetVoltUp(SubSys_Climber climberSubSys) {
    m_climberSubSys = climberSubSys;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberSubSys.setVoltCmdUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubSys.setVoltCmd(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
