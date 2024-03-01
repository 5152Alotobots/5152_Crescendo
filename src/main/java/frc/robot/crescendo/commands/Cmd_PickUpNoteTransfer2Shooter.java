// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.IntakeArm;
import frc.robot.crescendo.subsystems.intake.commands.Cmd_SubSys_Intake_ArmPosCmd;
import frc.robot.crescendo.subsystems.intake.commands.Cmd_SubSys_Intake_IntakeNote;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.ShooterArm;
import frc.robot.crescendo.subsystems.shooter.commands.Cmd_SubSys_Shooter_RotateToDegree;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class Cmd_PickUpNoteTransfer2Shooter extends SequentialCommandGroup {
  private final SubSys_Intake intakeSubSys;
  private final SubSys_Shooter shooterSubSys;

  public Cmd_PickUpNoteTransfer2Shooter(SubSys_Intake intakeSubSys, SubSys_Shooter shooterSubSys) {
    this.intakeSubSys = intakeSubSys;
    this.shooterSubSys = shooterSubSys;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new Cmd_SubSys_Shooter_RotateToDegree(shooterSubSys, ShooterArm.ShooterArmTransferPos),
        new SequentialCommandGroup(
          new Cmd_SubSys_Intake_ArmPosCmd(intakeSubSys, IntakeArm.IntakeArmPickupPos),
          new Cmd_SubSys_Intake_IntakeNote(intakeSubSys),
          new Cmd_SubSys_Intake_ArmPosCmd(intakeSubSys, IntakeArm.IntakeArmTransferPos))),
      new Cmd_TransferIntake2Shooter(shooterSubSys, intakeSubSys));
  }
}
//We still need to go forward to intake and backward so pathplanner is accurate