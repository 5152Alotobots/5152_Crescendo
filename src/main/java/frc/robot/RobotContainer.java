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
import frc.robot.Library.DriveTrains.Cmds_SubSys_DriveTrain.Cmd_SubSys_DriveTrain_JoysticDefault;
import frc.robot.DriverStation.SubSys_DriverStation;
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
  // private final PDPSubSys m_PDPSubSys = new PDPSubSys();

  // private final SubSys_LimeLight limeLightSubSys = new SubSys_LimeLight();

  // public final SubSys_MecanumDrive mecanumDriveSubSys = new SubSys_MecanumDrive();

  // public final SubSys_ColorSensor colorSubSys = new SubSys_ColorSensor();

  // public final SubSys_DistanceSensor distanceSubsys = new SubSys_DistanceSensor();
  // ---- Driver Station

  // ---- Driver Station
    public final SubSys_DriverStation driverStationSubSys = new SubSys_DriverStation();
  // SetUp Auto
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  //! Auto Commands Go here 
  // EXAMPLE COMMANDpublic final Command m_leftbluecharge = new Auto_leftbluecharge_Cmd(driveSubSys, gyroSubSys, limeLightSubSys);


  /*
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands

    /** ***** Control System Components */
 
    //         () -> driverStationSubSys.GetArmExtendAxis()));

    // handSubSys.setDefaultCommand(new Cmd_HandWithSensor(
    //  handSubSys,
    //  colorSubSys,
    //  distanceSubsys,
    //  () ->  driverStationSubSys.HandSensorBtn())
    // );
/**
 ** Mecanum Drive
 *mecanumDriveSubSys.setDefaultCommand(
    new Cmd_MecanumDriveDefault(
      mecanumDriveSubSys,
      () -> driverStationSubSys.DriveFwdAxis(),
      () -> driverStationSubSys.DriveStrAxis(),
      () -> driverStationSubSys.DriveRotAxis()
    )
  );
 */

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
        () -> driverStationSubSys.DrivePerfModeBActive()
      )
    );

    /**  Sendable Chooser
    * *EXAMPLE 
    * m_chooser.addOption("[BASIC] leftbluecharge", m_leftbluecharge);
    * m_chooser.addOption("[BASIC] leftblueescape", m_leftblueescape);
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
    driverStationSubSys.GyroResetButton.onTrue(new InstantCommand(gyroSubSys::zeroYaw, gyroSubSys));

    // Pose Reset Command Button
    driverStationSubSys.PoseResetButton.onTrue(
        new InstantCommand(driveSubSys::setPoseToOrigin, driveSubSys));

    /** 
     * Button Configuration Example  
     * driverStationSubSys.RainbowLEDColorButton.onTrue(
     *     new Cmd_SetBlingColorValue(
     *         blingSubSys,
     *         Const_Bling.Controllers.controller1,
     *         Const_Bling.Patterns.FixedPalette.RainbowRainbow));
    */

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
