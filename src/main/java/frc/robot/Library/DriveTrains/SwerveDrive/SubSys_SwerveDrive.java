// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Library.DriveTrains.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Robot;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;
import java.io.File;
import java.io.IOException;

import swervelib.SwerveController;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;

public class SubSys_SwerveDrive implements Subsystem {
  /**
   * Creates a new SubSys_SwerveDrive.
   */
  private SwerveDrive swerveDrive;
  static File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

  public SubSys_SwerveDrive() throws IOException {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive();
  }

  public SubSys_SwerveDrive(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(
            translation,
            rotation,
            fieldRelative,
            false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  public SwerveController getSwerveController() {
    return swerveDrive.getSwerveController();
  }
}

