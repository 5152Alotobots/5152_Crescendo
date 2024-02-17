/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Robot.Calibrations;
import frc.robot.chargedup.DriverStation;
import frc.robot.crescendo.HMIStation;
import frc.robot.crescendo.subsystems.climber.SubSys_Climber;
import frc.robot.crescendo.subsystems.climber.commands.climberSetVoltDown;
import frc.robot.crescendo.subsystems.climber.commands.climberSetVoltUp;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;
import frc.robot.crescendo.subsystems.shooter.util.DirectionUtils;
import frc.robot.crescendo.subsystems.slider.SubSys_Slider;
import frc.robot.crescendo.subsystems.intake.commands.Cmd_SubSys_Intake_Default;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.commands.Cmd_SubSys_Shooter_Default;
import frc.robot.library.drivetrains.mecanum.SubSys_MecanumDrive;
import frc.robot.library.drivetrains.mecanum.commands.Cmd_SubSys_MecanumDrive_JoystickDefault;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;
import frc.robot.library.drivetrains.swerve_ctre.Telemetry;

import static frc.robot.crescendo.HMIStation.Constants.DRIVER_ROT_DEADBAND;
import static frc.robot.crescendo.HMIStation.Constants.DRIVER_XY_DEADBAND;

enum RobotSelection {
    CRESCENDO,
    CHARGEDUP,
    GHETTOBOT
}
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Structure to switch between robots
    private static final RobotSelection ROBOT = RobotSelection.CRESCENDO;    // 2024 Robot
  private Command runAuto;
  // The robot's subsystems and commands are defined here...

   public RobotContainer() {

    // ##### CHARGEDUP_ROBOT_2023 #####
    final DriverStation driverStationSubSys;

    // ##### GHETTOBOT #####
    final SubSys_MecanumDrive mecanumDriveSubSys;

    // ##### CRESCENDO_ROBOT_2024 #####
    
    // ---- Power Distribution
    // private final PDPSubSys m_PDPSubSys = new PDPSubSys();
 
    // ---- Drive Subsystem
    // swerve_ctre
    final CommandSwerveDrivetrain drivetrain;
    //final SwerveRequest.RobotCentric drive;
    final SwerveRequest.FieldCentric drive;
    final Telemetry logger;
    final HMIStation hmiStation;
    final SubSys_Intake intakeSubSys;
    final SubSys_Shooter shooterSubSys;
    final SubSys_Climber climberSubSys;
    final SubSys_Slider sliderSubSys;

    // Switch Robots
       switch (ROBOT) {
           // ##### CHARGEDUP_ROBOT_2023 #####
           case CHARGEDUP:
        
            // ---- Drive Subsystem ----
            // swerve_ctre
            drivetrain = frc.robot.library.drivetrains.swerve_ctre.mk4il22023.TunerConstants_MK4iL2_2023.DriveTrain;
            
            drive = new SwerveRequest.FieldCentric()
                .withDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd * 0.1)
                .withRotationalDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxRotSpd * 0.1) // Add a 10% deadband
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

            logger = new Telemetry(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd);
            runAuto = drivetrain.getAutoPath("Tests");

            // ---- Human Machine Interface Station ----
            hmiStation = new HMIStation();

            // Configure the button bindings
            configureButtonBindingsChargedUpRobot2023(drivetrain, drive, logger, hmiStation);
            break;
    
        // ##### GHETTOBOT #####
        case GHETTOBOT:

            // ---- Drive Subsystem ----
            mecanumDriveSubSys = new SubSys_MecanumDrive();

            // ---- Intake Subsystem ----
            intakeSubSys = new SubSys_Intake();

            // ---- Shooter Subsystem ----
            shooterSubSys = new SubSys_Shooter();

            // ---- Driver Station ----
            driverStationSubSys = new DriverStation();

            configureButtonBindingsGhettoBot(mecanumDriveSubSys, intakeSubSys, shooterSubSys, driverStationSubSys);
            break;
        // ##### CRESCENDO_ROBOT_2024 #####
        default:

            // ---- Drive Subsystem ----
            drivetrain = frc.robot.library.drivetrains.swerve_ctre.mk4il32024.TunerConstants_MK4iL3_2024.DriveTrain;
            
            drive = new SwerveRequest.FieldCentric()
                    .withDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd * DRIVER_XY_DEADBAND)
                    .withRotationalDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxRotSpd * DRIVER_ROT_DEADBAND) // Add a 10% deadband
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
            logger = new Telemetry(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd);

            // ---- Human Machine Interface Station ----
            hmiStation = new HMIStation();

            // ---- Intake Subsystem ----
            intakeSubSys = new SubSys_Intake();

            // ---- Shooter Subsystem ----
            shooterSubSys = new SubSys_Shooter();
            
            sliderSubSys = new SubSys_Slider();

            // ---- Climber Subsystem ----
            climberSubSys = new SubSys_Climber();
            
            // Configure the button bindings
            configureButtonBindingsCrescendoRobot2024(drivetrain, drive, logger, hmiStation, intakeSubSys, shooterSubSys, sliderSubSys, climberSubSys);

            break;
    }  
  }
  

  private void configureButtonBindingsChargedUpRobot2023(
    CommandSwerveDrivetrain drivetrain,
    SwerveRequest.FieldCentric drive,
    Telemetry logger,
    HMIStation hmiStation){

        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> 
            drive.withVelocityX(hmiStation.driveFwdAxisRaw() * Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd) // Drive forward with negative Y (forward)
              .withVelocityY(hmiStation.driveStrAxisRaw() * Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd) // Drive left with negative X (left)
              .withRotationalRate(hmiStation.driveRotAxisRaw() * Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxRotSpd) // Drive counterclockwise with negative X (left)
        ));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void configureButtonBindingsGhettoBot(
    SubSys_MecanumDrive mecanumDriveSubSys,
    SubSys_Intake intakeSubSys,
    SubSys_Shooter shooterSubSys,
    DriverStation driverStationSubSys){
        mecanumDriveSubSys.setDefaultCommand(new Cmd_SubSys_MecanumDrive_JoystickDefault(
            mecanumDriveSubSys,
            driverStationSubSys::driveFwdAxisRaw,
            driverStationSubSys::driveStrAxisRaw,
            driverStationSubSys::driveRotAxisRaw));
  }

  private void configureButtonBindingsCrescendoRobot2024(
    CommandSwerveDrivetrain drivetrain,
    SwerveRequest.FieldCentric drive,
    Telemetry logger,
    HMIStation hmiStation,
    SubSys_Intake subSysIntake,
    SubSys_Shooter subSysShooter,
    SubSys_Slider subSysSlider,
    SubSys_Climber subSysClimber) {

    // -- Drivetrain --
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() -> 
        drive.withVelocityX(hmiStation.driveFwdAxisRaw() * Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd) // Drive forward with negative Y (forward)
             .withVelocityY(hmiStation.driveStrAxisRaw() * Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd) // Drive left with negative X (left)
             .withRotationalRate(hmiStation.driveRotAxisRaw() * Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxRotSpd) // Drive counterclockwise with negative X (left)
      ));
      drivetrain.registerTelemetry(logger::telemeterize);

    if (Utils.isSimulation()) {
        drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    // -- Intake --
    subSysIntake.setDefaultCommand(new Cmd_SubSys_Intake_Default(subSysIntake, hmiStation::intakeArmAxisRaw, hmiStation.intakeIn::getAsBoolean , hmiStation.intakeOut::getAsBoolean));

    // -- Shooter --
    subSysShooter.setDefaultCommand(new Cmd_SubSys_Shooter_Default(
        subSysShooter,
        hmiStation::shooterRotateAxisRaw,
        () -> DirectionUtils.toShooterDirection(hmiStation.shooterShoot),
        () -> DirectionUtils.toIntakeDirection(hmiStation.shooterIn, hmiStation.shooterOut)
    ));

      // -- Climber --
      hmiStation.climberUp.whileTrue(new climberSetVoltUp(subSysClimber));
      hmiStation.climberDn.whileTrue(new climberSetVoltDown(subSysClimber));

      hmiStation.sliderOut.onTrue(new InstantCommand(subSysSlider::extend));
      hmiStation.sliderIn.onTrue(new InstantCommand(subSysSlider::retract));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return auto.getAutoCommand();
    //return new Command()
        return runAuto;  
    }
}
