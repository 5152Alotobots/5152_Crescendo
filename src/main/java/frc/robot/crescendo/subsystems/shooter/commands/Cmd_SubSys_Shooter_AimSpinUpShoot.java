// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cmd_SubSys_Shooter_AimSpinUpShoot extends SequentialCommandGroup {
  /** Creates a new Cmd_SubSys_Shooter_AimSpinUpShoot. */
  public Cmd_SubSys_Shooter_AimSpinUpShoot(
    double shooterArmPosCmd,
    double shooterWheelsSpdCmd,
    SubSys_Shooter subSysShooter
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Cmd_SubSys_Shooter_AimSpinUp(shooterArmPosCmd,shooterWheelsSpdCmd,subSysShooter),
      new Cmd_SubSys_Shooter_AimSpinShoot(shooterArmPosCmd, shooterWheelsSpdCmd, subSysShooter)
    );
  }
}
