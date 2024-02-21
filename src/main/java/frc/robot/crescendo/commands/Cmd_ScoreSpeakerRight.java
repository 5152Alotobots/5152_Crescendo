// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class Cmd_ScoreSpeakerRight extends Command {
  /** Creates a new Score Speaker Command. 
   * This command will Start at the defined pose and aim, spin up and shoot
   * 1. Aim, Spin Up and Drive to shooting spot
   * 2. Shoot
   * 3. Stow Shooter and back away to defined pose
  */
  public Cmd_ScoreSpeakerRight() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
