// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup.commands.auto.tripleelement;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.library.vision.photonvision.commands.Cmd_NavigateToBestVisionTarget;
import frc.robot.library.vision.photonvision.SubSys_Photonvision_Constants;
import frc.robot.library.vision.photonvision.SubSys_Photonvision;
import frc.robot.library.drivetrains.commands.Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj;
import frc.robot.chargedup.subsystems.arm.SubSys_Arm;
import frc.robot.chargedup.subsystems.arm.commands.Cmd_SubSys_Arm_RotateAndExtend;
import frc.robot.chargedup.subsystems.bling.SubSys_Bling;
import frc.robot.chargedup.subsystems.hand.SubSys_Hand;
import frc.robot.library.drivetrains.SubSys_DriveTrain;
import frc.robot.library.gyroscopes.pigeon2.SubSys_PigeonGyro;

public class Auto_LinkHPBlue_Cmd extends SequentialCommandGroup {
  private final SubSys_DriveTrain m_DriveTrain;
  private final SubSys_PigeonGyro m_pigeonGyro;
  private final SubSys_Hand m_hand;
  private final SubSys_Arm m_Arm;
  private final SubSys_Photonvision m_photonvision;
  private final SubSys_Bling m_bling;

  /**
   * Auto mode sequence:
   *
   * <p>1. Deliver cone to high position<br>
   * 2. Drive to pickup cube<br>
   * 3. Pickup cube via vision<br>
   * 4. Drive to deliver cube<br>
   * 5. Deliver cube to high position<br>
   * 6. Drive to pickup cone<br>
   * 7. Pickup cone via vision<br>
   * 8. Drive to deliver cone<br>
   * 9. Deliver cone to high position<br>
   *
   * @param driveSubSys Drive subsystem
   * @param pigeonGyro Pigeon Gyro subsystem
   * @param handSubSys Hand subsystem
   * @param armSubSys Arm subsystem
   * @param photonvisionSubSys Photonvision subsystem
   * @param blingSubSys Bling subsystem
   */
  public Auto_LinkHPBlue_Cmd(
      SubSys_DriveTrain driveSubSys,
      SubSys_PigeonGyro pigeonGyro,
      SubSys_Hand handSubSys,
      SubSys_Arm armSubSys,
      SubSys_Photonvision photonvisionSubSys,
      SubSys_Bling blingSubSys) {
    m_DriveTrain = driveSubSys;
    m_pigeonGyro = pigeonGyro;
    m_hand = handSubSys;
    m_Arm = armSubSys;
    m_photonvision = photonvisionSubSys;
    m_bling = blingSubSys;

    /* Lift arm to high position from pickup sequential command group */
    SequentialCommandGroup armRotateAndExtendToHighLevelConeFromPickupSequential =
        new SequentialCommandGroup(
            new Cmd_SubSys_Arm_RotateAndExtend(armSubSys, -90, true, 0, false).withTimeout(4));

    /* Lift arm to high position for cube from pickup sequential command group */
    SequentialCommandGroup armRotateAndExtendToHighLevelCubeFromPickupSequential =
        new SequentialCommandGroup(
            new Cmd_SubSys_Arm_RotateAndExtend(armSubSys, -90, true, 0, false).withTimeout(4),
            new Cmd_SubSys_Arm_RotateAndExtend(armSubSys, -158.0, true, 1.54, true).withTimeout(6));

    /* Parallel commands */
    ParallelCommandGroup armRotateAndRetractDriveToPickup1Parallel =
        new ParallelCommandGroup(
            new Cmd_SubSys_Arm_RotateAndExtend(armSubSys, 0, true, 0.8, true)
                .withTimeout(4), // Lift arm to pickup pos
            new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(
                driveSubSys, "humanplayerside_pickup1_3element", true, true, Alliance.Blue));

    ParallelCommandGroup armRotateAndExtendDriveToDeliverCube1HighLevelParallel =
        new ParallelCommandGroup(
            armRotateAndExtendToHighLevelCubeFromPickupSequential.withTimeout(
                6), // Lift arm to high position
            new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(
                driveSubSys, "humanplayerside_deliver1_3element", false, false, Alliance.Blue));

    ParallelCommandGroup armRotateAndRetractDriveToPickup2Parallel =
        new ParallelCommandGroup(
            new Cmd_SubSys_Arm_RotateAndExtend(armSubSys, 0, true, 0.8, true)
                .withTimeout(4), // Lift arm to pickup pos
            new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(
                driveSubSys, "humanplayerside_pickup2_3element", false, false, Alliance.Blue));

    ParallelCommandGroup armRotateAndExtendDriveToDeliverCone2HighLevelParallel =
        new ParallelCommandGroup(
            armRotateAndExtendToHighLevelConeFromPickupSequential.withTimeout(
                6), // Lift arm to pickup pos
            new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(
                driveSubSys, "humanplayerside_deliver2_3element", false, false, Alliance.Blue));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new Cmd_SubSys_Arm_RotateAndExtend(armSubSys, -147.0, true, 1.54, true)
            .withTimeout(4), // Lift arm to high position
        new WaitCommand(0.5), // Add buffer time
        new InstantCommand(handSubSys::CloseHand),
        armRotateAndRetractDriveToPickup1Parallel,
        new Cmd_NavigateToBestVisionTarget(
                driveSubSys,
                m_photonvision,
                m_bling,
                SubSys_Photonvision_Constants.Cameras.frontCamera,
                SubSys_Photonvision_Constants.Pipelines.Cube,
                true,
                false,
                true)
            .withTimeout(3),
        new Cmd_SubSys_Arm_RotateAndExtend(armSubSys, 42, true, 0.8, true)
            .withTimeout(4), // Lift arm to pickup pos
        new InstantCommand(handSubSys::OpenHand),
        armRotateAndExtendDriveToDeliverCube1HighLevelParallel,
        new InstantCommand(handSubSys::CloseHand),
        new WaitCommand(0.5), // Add buffer time
        armRotateAndRetractDriveToPickup2Parallel,
        new Cmd_NavigateToBestVisionTarget(
                driveSubSys,
                m_photonvision,
                m_bling,
                SubSys_Photonvision_Constants.Cameras.frontCamera,
                SubSys_Photonvision_Constants.Pipelines.Cone,
                true,
                false,
                true)
            .withTimeout(5),
        new Cmd_SubSys_Arm_RotateAndExtend(armSubSys, 42, true, 0.8, true)
            .withTimeout(4), // Lift arm to pickup pos
        new InstantCommand(handSubSys::OpenHand),
        armRotateAndExtendDriveToDeliverCone2HighLevelParallel,
        new Cmd_SubSys_Arm_RotateAndExtend(armSubSys, -155.0, true, 1.54, true)
            .withTimeout(4), // TODO: Change value to whatever is on local driver station
        new WaitCommand(0.5), // Add buffer time
        new InstantCommand(handSubSys::CloseHand));
  }
}
