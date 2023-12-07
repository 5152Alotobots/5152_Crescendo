/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm.Cmd_SubSys_Arm_JoysticDefault;
import frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm.Cmd_SubSys_Arm_PosCmd;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.ChargedUp.Bling.Cmd.Cmd_SetBlingColorValue;
import frc.robot.ChargedUp.Bling.Const_Bling;
import frc.robot.ChargedUp.Bling.SubSys_Bling;
import frc.robot.ChargedUp.DriverStation.SubSys_DriverStation;
import frc.robot.ChargedUp.Hand.SubSys_Hand;
import frc.robot.ChargedUp.PhotonVision.SubSys_Photonvision;
import frc.robot.Library.DriveTrains.Cmds_SubSys_DriveTrain.Cmd_SubSys_DriveTrain_JoysticDefault;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;
import frc.robot.Library.Vision.Limelight.SubSys_LimeLight;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /** **** Library Components */

  // ---- Power Distribution
  // private final PDPSubSys m_PDPSubSys = new PDPSubSys();

  // ---- NavXGyro
  // public final NavXGyroSubSys m_NavXGyroSubSys = new NavXGyroSubSys();

  // ---- Pigeon2


  public final SubSys_PigeonGyro gyroSubSys = new SubSys_PigeonGyro();

  // ---- LimeLight
  private final SubSys_LimeLight limeLightSubSys = new SubSys_LimeLight();

  // ---- Drive Subsystem (Swerve)
  public final SubSys_DriveTrain driveSubSys = new SubSys_DriveTrain(gyroSubSys);

  public final SubSys_Photonvision photonvisionSubSys = new SubSys_Photonvision();

  // private final PDPSubSys m_PDPSubSys = new PDPSubSys();

  // private final SubSys_LimeLight limeLightSubSys = new SubSys_LimeLight();

  // public final SubSys_MecanumDrive mecanumDriveSubSys = new SubSys_MecanumDrive();

  // public final SubSys_ColorSensor colorSubSys = new SubSys_ColorSensor();

  // public final SubSys_DistanceSensor distanceSubsys = new SubSys_DistanceSensor();
  // ---- Driver Station

  // ---- Hand
  public final SubSys_Hand handSubSys = new SubSys_Hand();

  // Arm
  public final SubSys_Arm armSubSys = new SubSys_Arm(handSubSys.getHandLength());

  public final SubSys_Bling blingSubSys = new SubSys_Bling();

  public final Auto auto = new Auto();
  /*
   ***** Charged Up Componentes
   */

  // ---- Driver Station
  public final SubSys_DriverStation driverStationSubSys = new SubSys_DriverStation();

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands

    /** ***** Control System Components */
    armSubSys.setDefaultCommand(
        new Cmd_SubSys_Arm_JoysticDefault(
            armSubSys,
            () -> driverStationSubSys.GetArmRotateAxis(),
            () -> driverStationSubSys.GetArmExtendAxis()));

    // handSubSys.setDefaultCommand(new Cmd_HandWithSensor(
    //  handSubSys,
    //  colorSubSys,
    //  distanceSubsys,
    //  () ->  driverStationSubSys.HandSensorBtn())
    // );

    // mecanumDriveSubSys.setDefaultCommand(
    //    new Cmd_MecanumDriveDefault(
    //        mecanumDriveSubSys,
    //        () -> driverStationSubSys.DriveFwdAxis(),
    //        () -> driverStationSubSys.DriveStrAxis(),
    //        () -> driverStationSubSys.DriveRotAxis()));

    driveSubSys.setDefaultCommand(
        new Cmd_SubSys_DriveTrain_JoysticDefault(
            driveSubSys,
            () -> driverStationSubSys.DriveFwdAxis(),
            () -> driverStationSubSys.DriveStrAxis(),
            () -> driverStationSubSys.DriveRotAxis(),
            true,
            () -> driverStationSubSys.RotateLeftPt(),
            () -> driverStationSubSys.RotateRightPt(),
            () -> driverStationSubSys.DrivePerfModeAActive(),
            () -> driverStationSubSys.DrivePerfModeBActive()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}. Use this method to define your
   * button->command mappings. Buttons can be created by instantiating a {@link GenericHID} or one
   * of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Gyro Reset Command Button
    driverStationSubSys.OpenHandButton.onTrue(new InstantCommand(handSubSys::OpenHand, handSubSys));
    driverStationSubSys.CloseHandButton.onTrue(
        new InstantCommand(handSubSys::CloseHand, handSubSys));
    driverStationSubSys.GyroResetButton.onTrue(new InstantCommand(gyroSubSys::zeroYaw, gyroSubSys));

    // Gyro Reset Command Button
    driverStationSubSys.PoseResetButton.onTrue(
        // new InstantCommand(driveSubSys::setPoseToOrigin, driveSubSys));
        new InstantCommand(driveSubSys::setPoseToOrigin, driveSubSys));

    // Test Button
    driverStationSubSys.TestButton.whileTrue(
        new Cmd_SubSys_Arm_PosCmd(armSubSys, -145.0, true, 1.54, true)
        //   new CmdGrp_TestVisionAuto(driveSubSys, gyroSubSys, armSubSys, handSubSys, blingSubSys,
        // photonvisionSubSys)
        );

    driverStationSubSys.GroundPickupButton.whileTrue(
        new Cmd_SubSys_Arm_PosCmd(armSubSys, 45.0, true, 0.8, true));

    driverStationSubSys.HighConeDelivery.whileTrue(
        new Cmd_SubSys_Arm_PosCmd(armSubSys, -35.0, true, 1.65, true));

    driverStationSubSys.MidConeDelivery.whileTrue(
        new Cmd_SubSys_Arm_PosCmd(armSubSys, -25.0, true, 1.00, true));

    driverStationSubSys.HighSafePos.whileTrue(
        new Cmd_SubSys_Arm_PosCmd(armSubSys, -80.0, true, 0.8, true));

    // CONE/CUBE SIGNALING
    driverStationSubSys.RequestConeButton.onTrue(
        new Cmd_SetBlingColorValue(
            blingSubSys, Const_Bling.Controllers.controller1, Const_Bling.SolidColors.Yellow));
    driverStationSubSys.RequestCubeButton.onTrue(
        new Cmd_SetBlingColorValue(
            blingSubSys, Const_Bling.Controllers.controller1, Const_Bling.SolidColors.Violet));

    // Fun signaling
    driverStationSubSys.ResetLEDColorButton.onTrue(
        new Cmd_SetBlingColorValue(
            blingSubSys,
            Const_Bling.Controllers.controller1,
            Const_Bling.Patterns.Color1Color2.ColorWaves));
    driverStationSubSys.RainbowLEDColorButton.onTrue(
        new Cmd_SetBlingColorValue(
            blingSubSys,
            Const_Bling.Controllers.controller1,
            Const_Bling.Patterns.FixedPalette.RainbowRainbow));
    driverStationSubSys.RainbowStrobeLEDColorButton.onTrue(
        new Cmd_SetBlingColorValue(
            blingSubSys,
            Const_Bling.Controllers.controller1,
            Const_Bling.Patterns.FixedPalette.StrobeRed));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // m_DriveSubSys.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));

    // return m_BasicAutoCmd;
    // m_DriveSubSys.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));

    // return m_BasicAutoCmd;
    return auto.getAutoCommand();
  }
}
