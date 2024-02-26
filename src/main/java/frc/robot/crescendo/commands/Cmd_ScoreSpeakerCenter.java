// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cmd_ScoreSpeakerCenter extends SequentialCommandGroup {
  /** Creates a new Cmd_Shoot Speaker Center. 
   * This Command will start from the defined pose in front of the Amp, aim drive forward, score the note and back off.
   * 1. Aim the Shooter for the Speaker
   * 2. Drive forward
   * 3. Shoot the Note
   * 4. Drive backward
   * 5. Stow the Shooter
 * @param object3 
 * @param object2 
 * @param object 
 * @param shooterSubSys 
  */
  public Cmd_ScoreSpeakerCenter(SubSys_Shooter shooterSubSys, Object object, Object object2, Object object3) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
