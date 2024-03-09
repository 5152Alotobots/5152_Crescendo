// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;

import static frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.PresetIntakePositions.INTAKE_PRESET_PICKUP;
import static frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.PresetIntakePositions.INTAKE_PRESET_STOW;
import static frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.PresetIntakePositions.INTAKE_PRESET_TRANSFER;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cmd_SubSys_Intake_PickUpNote extends SequentialCommandGroup {
  /** Creates a new Cmd_SubSys_Intake_PickUpNote. */
  public Cmd_SubSys_Intake_PickUpNote(SubSys_Intake intakeSubSys) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
            new Cmd_SubSys_Intake_RotateToDegree(intakeSubSys, () -> INTAKE_PRESET_PICKUP),
            new Cmd_SubSys_Intake_IntakeNote(intakeSubSys),
            new Cmd_SubSys_Intake_RotateToDegree(intakeSubSys, () -> INTAKE_PRESET_TRANSFER));
  }
}
