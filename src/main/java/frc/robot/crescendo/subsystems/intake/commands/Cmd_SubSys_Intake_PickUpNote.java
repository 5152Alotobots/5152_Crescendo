// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class Cmd_SubSys_Intake_PickUpNote extends Command {
  /** Creates a new Cmd_PickUpNote. 
   * This Command will drop the Intake Arm, intake a note and lift the arm to ready to transfer position.
   * 1. Move Intake Arm to Pickup Position
   * 2. Turn on Intake Roller until the IR sensor detects the note or a timeout has occurred.
   * 3. Move towards note. 
   * 4. Upon IR detection or timeout move backwards to previous position.
   * 5. Move Intake Arm to transfer position.
  */


  public Cmd_SubSys_Intake_PickUpNote() {
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
