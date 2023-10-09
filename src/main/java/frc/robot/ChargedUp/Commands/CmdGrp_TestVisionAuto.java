// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
__          __           _                                      _          _                       _
\ \        / /          | |                          /\        | |        | |                     | |
 \ \  /\  / /_ _ _   _  | |_ ___     __ _  ___      /  \  _   _| |_ ___   | |_ ___  __ _ _ __ ___ | |
  \ \/  \/ / _` | | | | | __/ _ \   / _` |/ _ \    / /\ \| | | | __/ _ \  | __/ _ \/ _` | '_ ` _ \| |
   \  /\  / (_| | |_| | | || (_) | | (_| | (_) |  / ____ \ |_| | || (_) | | ||  __/ (_| | | | | | |_|
    \/  \/ \__,_|\__, |  \__\___/   \__, |\___/  /_/    \_\__,_|\__\___/   \__\___|\__,_|_| |_| |_(_)
                  __/ |              __/ |
                 |___/              |___/
*/
// WORKS - TESTED ON FIELD
package frc.robot.ChargedUp.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm.Cmd_SubSys_Arm_PosCmd;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.ChargedUp.Bling.SubSys_Bling;
import frc.robot.ChargedUp.Hand.SubSys_Hand;
import frc.robot.ChargedUp.PhotonVision.Const_Photonvision;
import frc.robot.ChargedUp.PhotonVision.SubSys_Photonvision;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.DriveTrains.SwerveDrive.*;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;

/**
 * *Link For PathPlaner *
 * https://docs.google.com/presentation/d/1xjYSI4KpbmGBUY-ZMf1nAFrXIoJo1tl-HHNl8LLqa1I/edit#slide=id.g1e65ac68f1d_0_48
 */
public class CmdGrp_TestVisionAuto extends SequentialCommandGroup {
  private final SubSys_DriveTrain m_DriveTrain;
  private final SubSys_PigeonGyro m_pigeonGyro;
  private final SubSys_Arm subsysArm;
  private final SubSys_Hand subsysHand;
  private final SubSys_Bling blingSubSys;
  private final SubSys_Photonvision photonvision;

  /** Creates a new Auto_Challenge1_Cmd. */
  public CmdGrp_TestVisionAuto(
      SubSys_DriveTrain driveSubSys,
      SubSys_PigeonGyro pigeonGyro,
      SubSys_Arm arm,
      SubSys_Hand hand,
      SubSys_Bling bling,
      SubSys_Photonvision photonvision) {
    m_DriveTrain = driveSubSys;
    m_pigeonGyro = pigeonGyro;
    subsysArm = arm;
    subsysHand = hand;
    blingSubSys = bling;
    this.photonvision = photonvision;

    /* Construct parallel command groups */

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(driveSubSys, "TestVisionAuto1", true,
        // true, Alliance.Red),
        new Cmd_SubSys_Arm_PosCmd(subsysArm, 0, true, 0.8, true).withTimeout(4),
        new Cmd_NavigateToBestVisionTarget(
                driveSubSys,
                photonvision,
                bling,
                Const_Photonvision.Cameras.frontCamera,
                Const_Photonvision.Pipelines.Cube,
                true,
                false,
                true)
            .withTimeout(3),
        new Cmd_SubSys_Arm_PosCmd(subsysArm, 40, true, 0.8, true).withTimeout(4),
        new InstantCommand(subsysHand::OpenHand, subsysHand),
        new Cmd_SubSys_Arm_PosCmd(subsysArm, 0, true, 0.8, true).withTimeout(4));
  }
}
