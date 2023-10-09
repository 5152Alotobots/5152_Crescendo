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
package frc.robot.ChargedUp.AutoCommands.SingleElement.Cube;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm.Cmd_SubSys_Arm_PosCmd;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.ChargedUp.Bling.Cmd.Cmd_SetBlingColorValue;
import frc.robot.ChargedUp.Bling.Const_Bling;
import frc.robot.ChargedUp.Bling.SubSys_Bling;
import frc.robot.ChargedUp.ChargeStation.Cmd_AutoBalance;
import frc.robot.ChargedUp.Commands.Cmd_NavigateToBestVisionTarget;
import frc.robot.ChargedUp.Hand.SubSys_Hand;
import frc.robot.ChargedUp.PhotonVision.Const_Photonvision;
import frc.robot.ChargedUp.PhotonVision.SubSys_Photonvision;
import frc.robot.Library.DriveTrains.Cmds_SubSys_DriveTrain.Cmds_PathPlanner.Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.DriveTrains.SwerveDrive.*;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;

/**
 * *Link For PathPlaner *
 * https://docs.google.com/presentation/d/1xjYSI4KpbmGBUY-ZMf1nAFrXIoJo1tl-HHNl8LLqa1I/edit#slide=id.g1e65ac68f1d_0_48
 */
public class Auto_Statebestcharge_blue_Cmd extends SequentialCommandGroup {
  private final SubSys_DriveTrain m_DriveTrain;
  private final SubSys_PigeonGyro m_pigeonGyro;
  private final SubSys_Arm subsysArm;
  private final SubSys_Hand subsysHand;
  private final SubSys_Bling blingSubSys;
  private final SubSys_Photonvision photonvision;

  /** Creates a new Auto_Challenge1_Cmd. */
  public Auto_Statebestcharge_blue_Cmd(
      SubSys_DriveTrain driveSubSys,
      SubSys_Arm arm,
      SubSys_Hand hand,
      SubSys_PigeonGyro pigeonGyro,
      SubSys_Bling bling,
      SubSys_Photonvision photonvision) {
    m_DriveTrain = driveSubSys;
    m_pigeonGyro = pigeonGyro;
    subsysArm = arm;
    subsysHand = hand;
    blingSubSys = bling;
    this.photonvision = photonvision;

    /* Construct parallel command groups */
    ParallelCommandGroup driveAndMoveToPickupPosition =
        new ParallelCommandGroup(
            new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(
                driveSubSys, "bestcharge1", true, true, Alliance.Blue),
            new Cmd_SubSys_Arm_PosCmd(subsysArm, 0.0, true, 0.8, true));

    ParallelCommandGroup driveAndRetractArm =
        new ParallelCommandGroup(
            new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(
                driveSubSys, "bestcharge2", false, false, Alliance.Blue),
            new Cmd_SubSys_Arm_PosCmd(subsysArm, 10.0, true, 0.8, true).withTimeout(4));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new Cmd_SubSys_Arm_PosCmd(subsysArm, -147.0, true, 1.54, true)
            .withTimeout(4), // Lift arm to high position
        new WaitCommand(1.5), // Add buffer time
        new InstantCommand(subsysHand::CloseHand, subsysHand), // Open hand (reversed)
        driveAndMoveToPickupPosition, // Drive to charge station whilst retracting arm
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
        new Cmd_SubSys_Arm_PosCmd(subsysArm, 40, true, 0.8, true),
        new InstantCommand(subsysHand::OpenHand, subsysHand), // Open hand (reversed)
        driveAndRetractArm,
        new Cmd_AutoBalance(pigeonGyro, driveSubSys), // Balance on charge station
        new Cmd_SetBlingColorValue(
            blingSubSys,
            Const_Bling.Controllers.controller1,
            Const_Bling.Patterns.FixedPalette.RainbowRainbow) // Celebrate!
        );
  }
}
