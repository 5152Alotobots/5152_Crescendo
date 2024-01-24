/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.library.vision.photonvision.SubSys_Photonvision;
import frc.robot.Constants.CodeConfigurations;
import frc.robot.chargedup.subsystems.arm.SubSys_Arm;
import frc.robot.chargedup.subsystems.bling.SubSys_Bling;
import frc.robot.chargedup.subsystems.hand.SubSys_Hand;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;
import frc.robot.crescendo.subsystems.intake.commands.Cmd_SubSys_Intake_Default;
import frc.robot.library.drivetrains.mecanum.SubSys_MecanumDrive;
import frc.robot.library.drivetrains.mecanum.commands.Cmd_SubSys_MecanumDrive_JoystickDefault;
import frc.robot.crescendo.DriverStation;
import frc.robot.library.drivetrains.swerve_5152.SubSys_DriveTrain;
import frc.robot.library.drivetrains.swerve_5152.commands.Cmd_SubSys_DriveTrain_JoysticDefault;
import frc.robot.library.drivetrains.swerve_yagsl.SubSysSwerveYAGSL;
import frc.robot.library.drivetrains.swerve_yagsl.commands.AbsoluteDriveAdv;
import frc.robot.library.gyroscopes.pigeon2.SubSys_PigeonGyro;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    public final DriverStation driverStationSubSys = new DriverStation();
    public final Auto auto;

    public RobotContainer() {
        final SubSys_PigeonGyro gyroSubSys;
        final SubSys_DriveTrain driveSubSys;
        final SubSys_MecanumDrive mecanumDriveSubSys;
        final SubSys_Photonvision photonvisionSubSys;
        final SubSys_Hand handSubSys;
        final SubSys_Arm armSubSys;
        final SubSys_Bling blingSubSys;
        final SubSys_Intake intakeSubSys;

        if (CodeConfigurations.SelectedDriveTrain == CodeConfigurations.DriveTrainOptions.SWERVE) {
            blingSubSys = new SubSys_Bling();
            photonvisionSubSys = new SubSys_Photonvision();
            handSubSys = new SubSys_Hand();
            armSubSys = new SubSys_Arm(0);
            gyroSubSys = new SubSys_PigeonGyro();
            driveSubSys = new SubSys_DriveTrain(gyroSubSys);
            configureButtonBindingsOldSwerve(handSubSys, gyroSubSys, driveSubSys);

            auto = new Auto(blingSubSys, photonvisionSubSys, handSubSys, armSubSys, gyroSubSys, driveSubSys);
        } else {
            mecanumDriveSubSys = new SubSys_MecanumDrive();
            intakeSubSys = new SubSys_Intake();
            configureButtonBindingsMecanum(mecanumDriveSubSys, intakeSubSys);
            auto = new Auto();
        }
    }

    private void configureButtonBindingsOldSwerve(
        SubSys_Hand handSubSys,
        SubSys_PigeonGyro gyroSubSys,
        SubSys_DriveTrain driveSubSys
        ) {

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
                        driverStationSubSys::DrivePerfModeBActive));
    }

    private void configureButtonBindingsMecanum(SubSys_MecanumDrive mecanumDriveSubSys, SubSys_Intake intakeSubSys) {
        mecanumDriveSubSys.setDefaultCommand(new Cmd_SubSys_MecanumDrive_JoystickDefault(
                mecanumDriveSubSys,
                driverStationSubSys::DriveFwdAxis,
                driverStationSubSys::DriveStrAxis,
                driverStationSubSys::DriveRotAxis));
        driverStationSubSys.TestButton.whileTrue(new Cmd_SubSys_Intake_Default(() -> (.5), intakeSubSys));
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
