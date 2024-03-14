// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands.expirimental;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.crescendo.subsystems.bling.SubSys_Bling;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.expirimental.Cmd_SubSys_Shooter_AutoShoot;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;
import frc.robot.library.drivetrains.swerve_ctre.commands.Cmd_SubSys_Drive_DriveWhileFacingSpeaker;

import java.util.function.DoubleSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cmd_ScoreSpeakerAutomatic extends ParallelCommandGroup {
  public Cmd_ScoreSpeakerAutomatic(
          CommandSwerveDrivetrain drivetrain,
          SubSys_Bling subSysBling,
          SubSys_Shooter subSysShooter,
          DoubleSupplier velocityX,
          DoubleSupplier velocityY) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
            new Cmd_SubSys_Drive_DriveWhileFacingSpeaker(drivetrain, velocityX, velocityY),
            new Cmd_SubSys_Shooter_AutoShoot(subSysShooter, subSysBling, drivetrain)
    );
  }
}
