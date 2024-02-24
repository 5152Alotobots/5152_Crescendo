// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.subsystems.climber.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.climber.SubSys_Climber;

public class Cmd_SubSys_Climber_Default extends Command {
  private final SubSys_Climber climberSubSys;
  private final BooleanSupplier climberUp;
  private final BooleanSupplier climberDn;

  public Cmd_SubSys_Climber_Default(
    BooleanSupplier climberUp, 
    BooleanSupplier climberDn, 
    SubSys_Climber climberSubSys) {
 
    this.climberSubSys = climberSubSys;
    this.climberUp = climberUp;
    this.climberDn = climberDn;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(climberUp.getAsBoolean()){
      climberSubSys.setVoltCmd(5);
    }else if(climberDn.getAsBoolean()){
      climberSubSys.setVoltCmd(-5);
    }else{
      climberSubSys.setVoltCmd(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubSys.setVoltCmd(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
