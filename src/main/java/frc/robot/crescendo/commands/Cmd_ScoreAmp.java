// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.commands.Cmd_SubSys_Shooter_RotateToDegree;
import frc.robot.crescendo.subsystems.shooter.commands.Cmd_SubSys_Shooter_ShootAtSpeed;

import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.PresentArmPositions.ARM_PRESET_AMP;
import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.Speeds.SHOOTER_AMP_SPEED;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cmd_ScoreAmp extends SequentialCommandGroup {
  /** Creates a new Cmd_ShootAmp. 
   * This Command will start from the defined pose in front of the Amp, aim drive forward, score the note and back off.
   * 1. Aim the Shooter for the Amp
   * 2. Drive forward
   * 3. Shoot the Note
   * 4. Drive backward
   * 5. Stow the Shooter
  */
  public Cmd_ScoreAmp(SubSys_Shooter shooterSubSys) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
            new Cmd_SubSys_Shooter_RotateToDegree(shooterSubSys, () -> ARM_PRESET_AMP),
            new Cmd_SubSys_Shooter_ShootAtSpeed(shooterSubSys, () -> SHOOTER_AMP_SPEED)
    );
  }
}
