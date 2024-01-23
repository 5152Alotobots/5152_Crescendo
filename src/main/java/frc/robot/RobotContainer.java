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
import frc.robot.library.vision.photonvision.SubSys_Photonvision;
import frc.robot.Constants.Robot.Calibrations.DriveTrain;
import frc.robot.Constants.CodeConfigurations;
import frc.robot.chargedup.DriverStation;
import frc.robot.chargedup.subsystems.arm.SubSys_Arm;
import frc.robot.chargedup.subsystems.arm.commands.Cmd_SubSys_Arm_JoysticDefault;
import frc.robot.chargedup.subsystems.arm.commands.Cmd_SubSys_Arm_RotateAndExtend;
import frc.robot.chargedup.subsystems.bling.SubSys_Bling;
import frc.robot.chargedup.subsystems.bling.SubSys_Bling_Constants;
import frc.robot.chargedup.subsystems.bling.commands.Cmd_SubSys_Bling_SetColorValue;
import frc.robot.chargedup.subsystems.hand.SubSys_Hand;
import frc.robot.library.drivetrains.mecanum.SubSys_MecanumDrive;
import frc.robot.library.drivetrains.mecanum.commands.Cmd_SubSys_MecanumDrive_JoystickDefault;
import frc.robot.library.drivetrains.swerve_5152.SubSys_DriveTrain;
import frc.robot.library.drivetrains.swerve_5152.commands.Cmd_SubSys_DriveTrain_JoysticDefault;
import frc.robot.library.gyroscopes.pigeon2.SubSys_PigeonGyro;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
 
  public final SubSys_PigeonGyro gyroSubSys = new SubSys_PigeonGyro();
  public final SubSys_DriveTrain driveSubSys = new SubSys_DriveTrain(gyroSubSys);
  public final SubSys_MecanumDrive mecanumDriveSubSys = new SubSys_MecanumDrive();

  public final SubSys_Photonvision photonvisionSubSys = new SubSys_Photonvision();

  public SubSys_Hand handSubSys; //= new SubSys_Hand();
  public SubSys_Arm armSubSys; //= new SubSys_Arm(handSubSys.getHandLength());

  public final SubSys_Bling blingSubSys = new SubSys_Bling();



  public final DriverStation driverStationSubSys = new DriverStation();
  public final Auto auto;
  public RobotContainer() {
    if (Boolean.FALSE.equals(CodeConfigurations.DISABLE_AUTO)) {
        auto = new Auto(blingSubSys, photonvisionSubSys, handSubSys, armSubSys, gyroSubSys, driveSubSys);
    } else {
        auto = new Auto();
    }
    // Configure the button bindings
    configureButtonBindings();
        if (CodeConfigurations.SelectedDriveTrain == CodeConfigurations.DriveTrainOptions.SWERVE) {
         driveSubSys.setDefaultCommand(
                new Cmd_SubSys_DriveTrain_JoysticDefault(
                    driveSubSys,
                        driverStationSubSys::DriveFwdAxis,
                        driverStationSubSys::DriveStrAxis,
                        driverStationSubSys::DriveRotAxis,
                    true,
                        driverStationSubSys::RotateLeftPt,
                        driverStationSubSys::RotateRightPt,
                        driverStationSubSys::DrivePerfModeAActive,
                        driverStationSubSys::DrivePerfModeBActive
                    )
                );
        } else if (CodeConfigurations.SelectedDriveTrain == CodeConfigurations.DriveTrainOptions.MECANUM) {
            mecanumDriveSubSys.setDefaultCommand(new Cmd_SubSys_MecanumDrive_JoystickDefault(
                mecanumDriveSubSys, 
                driverStationSubSys::DriveFwdAxis, 
                driverStationSubSys::DriveStrAxis, 
                driverStationSubSys::DriveRotAxis)
            );
        }
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
   // driverStationSubSys.OpenHandButton.onTrue(new InstantCommand(handSubSys::OpenHand, handSubSys));
   // driverStationSubSys.CloseHandButton.onTrue(
   //     new InstantCommand(handSubSys::CloseHand, handSubSys));
    driverStationSubSys.GyroResetButton.onTrue(new InstantCommand(gyroSubSys::zeroYaw, gyroSubSys));

    // Gyro Reset Command Button
    driverStationSubSys.PoseResetButton.onTrue(
        // new InstantCommand(driveSubSys::setPoseToOrigin, driveSubSys));
        new InstantCommand(driveSubSys::setPoseToOrigin, driveSubSys));

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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auto.getAutoCommand();
  }
}
