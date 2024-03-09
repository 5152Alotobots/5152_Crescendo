// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.commands.Cmd_SubSys_Shooter_Shoot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cmd_ScoreSpeakerLeft extends SequentialCommandGroup {
  /** Creates a new Cmd_Shoot Speaker Left. 
   * This Command will start from the defined pose in front of the Amp, aim drive forward, score the note and back off.
   * 1. Aim the Shooter for the Speaker
   * 2. Drive forward
   * 3. Shoot the Note
   * 4. Drive backward
   * 5. Stow the Shooter
  */
  public Cmd_ScoreSpeakerLeft(SubSys_Shooter shooterSubSys) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new Cmd_SubSys_Shooter_Shoot(shooterSubSys, () -> false).withTimeout(3));
  }
}
