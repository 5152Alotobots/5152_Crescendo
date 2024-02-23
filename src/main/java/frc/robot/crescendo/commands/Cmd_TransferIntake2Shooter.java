// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cmd_TransferIntake2Shooter extends SequentialCommandGroup {
  /** Creates Command to Transfer Note from the Shooter to the Shooter 
   * This Command will start from the defined pose in front of the Amp, aim drive forward, score the note and back off.
   * 1. Aim the Shooter for the Source
   * 2. Drive forward
   * 3. Shooter Intake the note
   * 4. Shooter Roller position the note
   * 4. Drive backward
   * 5. Stow the Shooter
  */
  public Cmd_TransferIntake2Shooter() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
