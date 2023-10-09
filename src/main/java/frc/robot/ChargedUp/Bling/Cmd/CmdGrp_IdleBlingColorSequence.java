// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Bling.Cmd;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ChargedUp.Bling.Const_Bling.Patterns;
import frc.robot.ChargedUp.Bling.Const_Bling.SolidColors;
import frc.robot.ChargedUp.Bling.SubSys_Bling;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CmdGrp_IdleBlingColorSequence extends SequentialCommandGroup {
  private SubSys_Bling subSys_Bling;
  private Spark controller;
  /** Creates a new CmdGrp_IdleBlingColorSequence. */
  public CmdGrp_IdleBlingColorSequence(SubSys_Bling subSys_Bling, Spark Controller) {
    this.subSys_Bling = subSys_Bling;
    this.controller = Controller;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new Cmd_SetBlingColorValue(subSys_Bling, controller, Patterns.FixedPalette.StrobeRed),
        new WaitCommand(5),
        new Cmd_SetBlingColorValue(subSys_Bling, controller, SolidColors.DarkRed));
  }
}
