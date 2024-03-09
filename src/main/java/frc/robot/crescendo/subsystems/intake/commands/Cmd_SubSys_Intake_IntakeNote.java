// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.intake.IntakeDirection;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;

public class Cmd_SubSys_Intake_IntakeNote extends Command {
  /** Creates a new Cmd_SubSys_Intake_IntakeNote. */
  private final SubSys_Intake intakeSubSys;

  public Cmd_SubSys_Intake_IntakeNote(SubSys_Intake intakeSubSys) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubSys = intakeSubSys;
    addRequirements(intakeSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubSys.setIntakeDirection(IntakeDirection.IN);
    // Code to drive forward into note
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubSys.setIntakeDirection(IntakeDirection.NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSubSys.getIntakeOccupied();
  }
}
