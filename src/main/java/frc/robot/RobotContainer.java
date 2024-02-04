/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;

import com.ctre.phoenix.Logger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.library.vision.photonvision.SubSys_Photonvision;
import frc.robot.Constants.Robot.Calibrations;
import frc.robot.chargedup.subsystems.arm.SubSys_Arm;
import frc.robot.chargedup.subsystems.bling.SubSys_Bling;
import frc.robot.chargedup.subsystems.bling.SubSys_Bling_Constants;
import frc.robot.chargedup.subsystems.bling.commands.Cmd_SubSys_Bling_SetColorValue;
import frc.robot.chargedup.subsystems.hand.SubSys_Hand;
import frc.robot.crescendo.HMIStation;
import frc.robot.library.drivetrains.swerve_5152.SubSys_DriveTrain;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;
import frc.robot.library.drivetrains.swerve_ctre.Telemetry;
import frc.robot.library.drivetrains.swerve_yagsl.SubSysSwerveYAGSL;
import frc.robot.library.drivetrains.swerve_yagsl.commands.AbsoluteDriveAdv;
import frc.robot.library.gyroscopes.pigeon2.SubSys_PigeonGyro;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /** **** Start Library Components **** */

  // ---- Power Distribution
  // private final PDPSubSys m_PDPSubSys = new PDPSubSys();

  // ---- Drive Subsystem
  // swerve_ctre
  private final CommandSwerveDrivetrain drivetrain = frc.robot.library.drivetrains.swerve_ctre.mk4il32024.TunerConstants_MK4iL3_2024.DriveTrain; // My drivetrain

//  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
      .withDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd * 0.1)
      .withRotationalDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxRotSpd * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd);

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  // swerve_yagsl
  //private final SubSysSwerveYAGSL drivebase = new SubSysSwerveYAGSL(new File(Filesystem.getDeployDirectory(),
  //                                                                       "swerve/mk4italonsl2"));

  // swerve_5152
  //public final SubSys_DriveTrain driveSubSys = new SubSys_DriveTrain(gyroSubSys);
   //public final SubSys_PigeonGyro gyroSubSys = new SubSys_PigeonGyro();

  // Mecanum
  // public final SubSys_MecanumDrive mecanumDriveSubSys = new SubSys_MecanumDrive();

  // ---- Photonvision
  //public final SubSys_Photonvision photonvisionSubSys = new SubSys_Photonvision();

  // private final SubSys_LimeLight limeLightSubSys = new SubSys_LimeLight();
  
  /** **** End Library Components **** */

  /** **** Start Crescendo Components **** */

  // ---- Driver Station ----
  public final HMIStation driverStationSubSys = new HMIStation();

  // ---- Intake ----

  // ---- Slider ----

  // ---- Shooter ----

  // ---- Climber ----

  /** **** End Crescendo Components **** */

  /** **** Auto **** */
  //public Auto auto;

  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();

    /** <<<< Start Default Commands >>>> */
    
    /** **** Start Library Components Default Commands **** */ 
    /** **** EndLibrary Components Default Commands **** */ 
    
    /** **** Start Crescendo Components Default Commands **** */
    /** **** End Crescendo Components Default Commands **** */

    /** <<<< End Default Commands >>>> */

    //auto = new Auto(blingSubSys, photonvisionSubSys, handSubSys, armSubSys, gyroSubSys, driveSubSys);
    
    
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

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> 
            drive.withVelocityX(driverStationSubSys.DriveFwdAxis() * Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd) // Drive forward with negative Y (forward)
              .withVelocityY(driverStationSubSys.DriveStrAxis() * Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd) // Drive left with negative X (left)
              .withRotationalRate(driverStationSubSys.DriveRotAxis() * Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxRotSpd) // Drive counterclockwise with negative X (left)
        ));

    //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    //joystick.b().whileTrue(drivetrain
    //    .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    //joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
        drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // Gyro Reset Command Button
    //driverStationSubSys.OpenHandButton.onTrue(new InstantCommand(handSubSys::OpenHand, handSubSys));
    //driverStationSubSys.CloseHandButton.onTrue(
    //    new InstantCommand(handSubSys::CloseHand, handSubSys));
    //driverStationSubSys.GyroResetButton.onTrue(new InstantCommand(gyroSubSys::zeroYaw, gyroSubSys));

    // Gyro Reset Command Button
    //driverStationSubSys.PoseResetButton.onTrue(
        // new InstantCommand(driveSubSys::setPoseToOrigin, driveSubSys));
    //    new InstantCommand(driveSubSys::setPoseToOrigin, driveSubSys));

    /*    
    // Test Button
    driverStationSubSys.TestButton.whileTrue(
        new Cmd_SubSys_Arm_RotateAndExtend(armSubSys, -145.0, true, 1.54, true)
        //   new CmdGrp_TestVisionAuto(driveSubSys, gyroSubSys, armSubSys, handSubSys, blingSubSys,
        // photonvisionSubSys)
        );
    

    driverStationSubSys.GroundPickupButton.whileTrue(
        new Cmd_SubSys_Arm_RotateAndExtend(armSubSys, 45.0, true, 0.8, true));

    driverStationSubSys.HighConeDelivery.whileTrue(
        new Cmd_SubSys_Arm_RotateAndExtend(armSubSys, -35.0, true, 1.65, true));

    driverStationSubSys.MidConeDelivery.whileTrue(
        new Cmd_SubSys_Arm_RotateAndExtend(armSubSys, -25.0, true, 1.00, true));

    driverStationSubSys.HighSafePos.whileTrue(
        new Cmd_SubSys_Arm_RotateAndExtend(armSubSys, -80.0, true, 0.8, true));
    */

    // CONE/CUBE SIGNALING
    /* 
    driverStationSubSys.RequestConeButton.onTrue(
        new Cmd_SubSys_Bling_SetColorValue(
            blingSubSys, SubSys_Bling_Constants.Controllers.controller1, SubSys_Bling_Constants.SolidColors.Yellow));
    driverStationSubSys.RequestCubeButton.onTrue(
        new Cmd_SubSys_Bling_SetColorValue(
            blingSubSys, SubSys_Bling_Constants.Controllers.controller1, SubSys_Bling_Constants.SolidColors.Violet));

    // Fun signaling
    driverStationSubSys.ResetLEDColorButton.onTrue(
        new Cmd_SubSys_Bling_SetColorValue(
            blingSubSys,
            SubSys_Bling_Constants.Controllers.controller1,
            SubSys_Bling_Constants.Patterns.Color1Color2.ColorWaves));
    driverStationSubSys.RainbowLEDColorButton.onTrue(
        new Cmd_SubSys_Bling_SetColorValue(
            blingSubSys,
            SubSys_Bling_Constants.Controllers.controller1,
            SubSys_Bling_Constants.Patterns.FixedPalette.RainbowRainbow));
    driverStationSubSys.RainbowStrobeLEDColorButton.onTrue(
        new Cmd_SubSys_Bling_SetColorValue(
            blingSubSys,
            SubSys_Bling_Constants.Controllers.controller1,
            SubSys_Bling_Constants.Patterns.FixedPalette.StrobeRed));
    */
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return auto.getAutoCommand();
    //return new Command() {
        return runAuto;  
    }
}
