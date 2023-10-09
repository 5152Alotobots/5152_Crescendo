/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm.Cmd_SubSys_Arm_JoysticDefault;
import frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm.Cmd_SubSys_Arm_PosCmd;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.ChargedUp.AutoCommands.Basic.*;
import frc.robot.ChargedUp.AutoCommands.DoubleElement.*;
import frc.robot.ChargedUp.AutoCommands.SingleElement.Cone.*;
import frc.robot.ChargedUp.AutoCommands.SingleElement.Cube.*;
import frc.robot.ChargedUp.AutoCommands.TripleElement.*;
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
  /*
   ***** Charged Up Componentes
   */

  // ---- Driver Station
  public final SubSys_DriverStation driverStationSubSys = new SubSys_DriverStation();
  // SetUp Auto
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final Command m_leftbluecharge = new Auto_leftbluecharge_Cmd(driveSubSys, gyroSubSys);

  private final Command m_leftblueescape = new Auto_leftblueescape_Cmd(driveSubSys, gyroSubSys);

  private final Command m_leftredcharge = new Auto_leftredcharge_Cmd(driveSubSys, gyroSubSys);

  private final Command m_leftredescape = new Auto_leftredescape_Cmd(driveSubSys, gyroSubSys);

  private final Command m_middlebluecharge = new Auto_middlebluecharge_Cmd(driveSubSys, gyroSubSys);

  private final Command m_middleredcharge = new Auto_middleredcharge_Cmd(driveSubSys, gyroSubSys);

  private final Command m_rightbluecharge = new Auto_rightbluecharge_Cmd(driveSubSys, gyroSubSys);

  private final Command m_stateescape = new Auto_stateescape_Cmd(driveSubSys, gyroSubSys);

  private final Command m_rightredcharge = new Auto_rightredcharge_Cmd(driveSubSys, gyroSubSys);

  private final Command m_rightredescape = new Auto_rightredescape_Cmd(driveSubSys, gyroSubSys);

  private final Command m_rightbluecone =
      new Auto_rightbluecone_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys);

  private final Command m_leftbluecone =
      new Auto_leftbluecone_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys);

  private final Command m_middlebluecone =
      new Auto_middlebluecone_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys);

  private final Command m_middleblueconeleftescape =
      new Auto_middleblueconeleftescape_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys);

  private final Command m_rightredcone =
      new Auto_rightredcone_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys);

  private final Command m_leftredcone =
      new Auto_leftredcone_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys);

  private final Command m_middleredcone =
      new Auto_middleredcone_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys);

  private final Command m_middleredconeleftescape =
      new Auto_middleredconeleftescape_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys);

  // ! NEW SINGLE ELEMENT COMMANDS
  // Cones
  private final Command m_leftbluecharge_1cone =
      new Auto_leftbluecharge_1cone_Cmd(
          driveSubSys, gyroSubSys, armSubSys, handSubSys, blingSubSys);

  private final Command m_leftblueescape_1cone =
      new Auto_leftblueescape_1cone_Cmd(
          driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys);

  private final Command m_leftredcharge_1cone =
      new Auto_leftredcharge_1cone_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys, blingSubSys);

  private final Command m_leftredescape_1cone =
      new Auto_leftredescape_1cone_Cmd(driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys);
  /*
  private final Command m_middlebluecharge_1cone =
      new Auto_middlebluecharge_1cone_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys, blingSubSys);
  */
  /*  private final Command m_middleredcharge_1cone =
      new Auto_middleredcharge_1cone_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys, blingSubSys);
  */
  private final Command m_rightbluecharge_1cone =
      new Auto_rightbluecharge_1cone_Cmd(
          driveSubSys, gyroSubSys, armSubSys, handSubSys, blingSubSys);

  private final Command m_rightblueescape_1cone =
      new Auto_rightblueescape_1cone_Cmd(
          driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys);

  private final Command m_rightredcharge_1cone =
      new Auto_rightredcharge_1cone_Cmd(
          driveSubSys, gyroSubSys, armSubSys, handSubSys, blingSubSys);

  private final Command m_rightredescape_1cone =
      new Auto_rightredescape_1cone_Cmd(
          driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys);

  // Cubes
  private final Command m_lakeview_1cube =
      new Auto_Lakeview_1cube_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys, blingSubSys);

  private final Command Auto_leftbluecharge_1cube_Cmd =
      new Auto_leftbluecharge_1cube_Cmd(
          driveSubSys, gyroSubSys, armSubSys, handSubSys, blingSubSys);

  private final Command Auto_leftblueescape_1cube_Cmd =
      new Auto_leftblueescape_1cube_Cmd(
          driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys);

  private final Command Auto_leftredcharge_1cube_Cmd =
      new Auto_leftredcharge_1cube_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys, blingSubSys);

  private final Command Auto_leftredescape_1cube_Cmd =
      new Auto_leftredescape_1cube_Cmd(driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys);

  private final Command Auto_rightbluecharge_1cube_Cmd =
      new Auto_rightbluecharge_1cube_Cmd(
          driveSubSys, gyroSubSys, armSubSys, handSubSys, blingSubSys);

  private final Command Auto_rightblueescape_1cube_Cmd =
      new Auto_rightblueescape_1cube_Cmd(
          driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys);

  private final Command Auto_rightredcharge_1cube_Cmd =
      new Auto_rightredcharge_1cube_Cmd(
          driveSubSys, gyroSubSys, armSubSys, handSubSys, blingSubSys);

  private final Command Auto_rightredescape_1cube_Cmd =
      new Auto_rightredescape_1cube_Cmd(
          driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys);

  private final Command Auto_Statemiddleleave_1cube_Cmd =
      new Auto_Statemiddleleave_1cube_Cmd(
          driveSubSys, gyroSubSys, armSubSys, handSubSys, blingSubSys);

  // Playoffs
  private final Command Auto_Statebarrier_1cone1cube_red_Cmd =
      new Auto_Statebarrier_1cone1cube_red_Cmd(
          driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys);

  private final Command Auto_Statebarrier_1cone1cube_blue_Cmd =
      new Auto_Statebarrier_1cone1cube_blue_Cmd(
          driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys);

  private final Command Auto_autolink_red_Cmd =
      new Auto_autolink_red_Cmd(driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys);

  private final Command Auto_vision_blue_Cmd =
      new Auto_vision_blue_Cmd(
          driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys, photonvisionSubSys);

  private final Command Auto_1cone2CubeHPBlue_Cmd =
      new Auto_1cone2cubeHPBlue_Cmd(
          driveSubSys, gyroSubSys, handSubSys, armSubSys, photonvisionSubSys, blingSubSys);

  private final Command Auto_1cone2CubeHPRed_Cmd =
      new Auto_1cone2cubeHPRed_Cmd(
          driveSubSys, gyroSubSys, handSubSys, armSubSys, photonvisionSubSys, blingSubSys);

  private final Command Auto_vision_red_Cmd =
      new Auto_vision_red_Cmd(
          driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys, photonvisionSubSys);

  private final Command Auto_Statebestcharge_blue_Cmd =
      new Auto_Statebestcharge_blue_Cmd(
          driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys, photonvisionSubSys);

  private final Command Auto_1cone2cubestashHPBlue_Cmd =
      new Auto_1cone2cubestashHPBlue_Cmd(
          driveSubSys, gyroSubSys, handSubSys, armSubSys, photonvisionSubSys, blingSubSys);

  private final Command Auto_1cone2cubestashHPRed_Cmd =
      new Auto_1cone2cubestashHPRed_Cmd(
          driveSubSys, gyroSubSys, handSubSys, armSubSys, photonvisionSubSys, blingSubSys);

  /*
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
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

    // Sendable Chooser
    /*  m_chooser.addOption("[BASIC] leftbluecharge", m_leftbluecharge);
    m_chooser.addOption("[BASIC] leftblueescape", m_leftblueescape);
    m_chooser.addOption("[BASIC] leftredcharge", m_leftredcharge);
    m_chooser.addOption("[BASIC] leftredescape", m_leftredescape);
    m_chooser.addOption("[BASIC] middlebluecharge", m_middlebluecharge);
    m_chooser.addOption("[BASIC] middleredcharge", m_middleredcharge);
    m_chooser.setDefaultOption("[BASIC] rightbluecharge", m_rightbluecharge);*/
    m_chooser.addOption("[BASIC] stateescape", m_stateescape);
    /* m_chooser.addOption("[BASIC] rightredcharge", m_rightredcharge);
    m_chooser.addOption("[BASIC] rightredescape", m_rightredescape);*/
    // NEW SINGLE ELEMENT COMMANDS
    // Cones
    /*   m_chooser.addOption("[SINGLE] leftbluecharge_1cone", m_leftbluecharge_1cone);
    m_chooser.addOption("[SINGLE] leftblueescape_1cone", m_leftblueescape_1cone);
    m_chooser.addOption("[SINGLE] leftredcharge_1cone", m_leftredcharge_1cone);
    m_chooser.addOption("[SINGLE] leftredescape_1cone", m_leftredescape_1cone);
    // m_chooser.addOption("[SINGLE] middlebluecharge_1cone", m_middlebluecharge_1cone);
    // m_chooser.addOption("[SINGLE] middleredcharge_1cone", m_middleredcharge_1cone);
    m_chooser.addOption("[SINGLE] rightbluecharge_1cone", m_rightbluecharge_1cone);*/
    m_chooser.addOption("[SINGLE] rightblueescape_1cone", m_rightblueescape_1cone);
    /*  m_chooser.addOption("[SINGLE] rightredcharge_1cone", m_rightredcharge_1cone);
    m_chooser.addOption("[SINGLE] rightredescape_1cone", m_rightredescape_1cone);
    // Cubes
    m_chooser.addOption("[SINGLE] leftbluecharge_1cube", Auto_leftbluecharge_1cube_Cmd);
    m_chooser.addOption("[SINGLE] leftblueescape_1cube", Auto_leftblueescape_1cube_Cmd);
    m_chooser.addOption("[SINGLE] leftredcharge_1cube", Auto_leftredcharge_1cube_Cmd);
    m_chooser.addOption("[SINGLE] leftredescape_1cube", Auto_leftredescape_1cube_Cmd);
    m_chooser.addOption("[SINGLE] rightbluecharge_1cube", Auto_rightbluecharge_1cube_Cmd);
    m_chooser.addOption("[SINGLE] rightblueescape_1cube", Auto_rightblueescape_1cube_Cmd);
    m_chooser.addOption("[SINGLE] rightredcharge_1cube", Auto_rightredcharge_1cube_Cmd);
    m_chooser.addOption("[SINGLE] rightredescape_1cube", Auto_rightredescape_1cube_Cmd);

    // LAKE VIEW
    m_chooser.addOption("lakeview", m_lakeview_1cube);*/
    m_chooser.addOption("Statemiddleleave_1cube", Auto_Statemiddleleave_1cube_Cmd);

    // Playoffs
    m_chooser.addOption("StateBarrier - Red", Auto_Statebarrier_1cone1cube_red_Cmd);
    m_chooser.addOption("StateBarrier - Blue", Auto_Statebarrier_1cone1cube_blue_Cmd);
    m_chooser.addOption("autolink - Red", Auto_autolink_red_Cmd);
    m_chooser.addOption("vision - blue", Auto_vision_blue_Cmd);
    m_chooser.addOption("vision - red", Auto_vision_red_Cmd);
    m_chooser.addOption("Statebestcharge - blue", Auto_Statebestcharge_blue_Cmd);
    m_chooser.addOption("[TRIPLE] 1 cone, 2 cube stash - Blue", Auto_1cone2cubestashHPBlue_Cmd);
    m_chooser.addOption("[TRIPLE] 1 cone, 2 cube stash - Red", Auto_1cone2cubestashHPRed_Cmd);
    m_chooser.addOption(
        "[TRIPLE] 1 cone, 2 cube Human Player Side - Blue", Auto_1cone2CubeHPBlue_Cmd);
    m_chooser.addOption(
        "[TRIPLE] 1 cone, 2 cube Human Player Side - Red", Auto_1cone2CubeHPRed_Cmd);

    // DOUBLE ELEMENT COMMANDS
    /*
    m_chooser.addOption("[DOUBLE] leftbluecone", m_leftbluecone);
    m_chooser.addOption("[DOUBLE] middlebluecone", m_middlebluecone);
    m_chooser.addOption("[DOUBLE] rightbluecone", m_rightbluecone);
    m_chooser.addOption("[DOUBLE] middleblueconeleftescape", m_middleblueconeleftescape);
    m_chooser.addOption("[DOUBLE] leftredcone", m_leftredcone);
    m_chooser.addOption("[DOUBLE] middleredcone", m_middleredcone);
    m_chooser.addOption("[DOUBLE] rightredcone", m_rightredcone);
    m_chooser.addOption("[DOUBLE] middleredconeleftescape", m_middleredconeleftescape);
    */
    SmartDashboard.putData(m_chooser);
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
    return m_chooser.getSelected();
  }
}
