// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class Cmd_ScoreAmp extends Command {
  /** Creates a new Cmd_ShootAmp. 
   * This Command will start from the defined pose in front of the Amp, aim drive forward, score the note and back off.
   * 1. Aim the Shooter for the Amp
   * 2. Drive forward
   * 3. Shoot the Note
   * 4. Drive backward
   * 5. Stow the Shooter
  */
  public Cmd_ScoreAmp() {
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
