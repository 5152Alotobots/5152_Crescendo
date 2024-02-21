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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Robot.Calibrations;
import frc.robot.chargedup.DriverStation;
import frc.robot.crescendo.HMIStation;
import frc.robot.crescendo.commands.Cmd_ScoreAmp;
import frc.robot.crescendo.commands.Cmd_ScoreSpeakerCenter;
import frc.robot.crescendo.commands.Cmd_ScoreSpeakerLeft;
import frc.robot.crescendo.commands.Cmd_ScoreSpeakerRight;
import frc.robot.crescendo.subsystems.climber.SubSys_Climber;
import frc.robot.crescendo.subsystems.climber.commands.climberSetVoltDn;
import frc.robot.crescendo.subsystems.climber.commands.climberSetVoltUp;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;
import frc.robot.crescendo.subsystems.intake.commands.Cmd_SubSys_Intake_Default;
import frc.robot.crescendo.subsystems.intake.commands.Cmd_SubSys_Intake_PickUpNote;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.commands.Cmd_SubSys_Shooter_Default;
import frc.robot.crescendo.subsystems.slider.SubSys_Slider;
import frc.robot.library.drivetrains.mecanum.SubSys_MecanumDrive;
import frc.robot.library.drivetrains.mecanum.commands.Cmd_SubSys_MecanumDrive_JoystickDefault;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;
import frc.robot.library.drivetrains.swerve_ctre.Telemetry;

import static frc.robot.crescendo.HMIStation.Constants.DRIVER_ROT_DEADBAND;
import static frc.robot.crescendo.HMIStation.Constants.DRIVER_XY_DEADBAND;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Structure to switch between robots  
  private static final int CRESCENDO_ROBOT_2024 = 24;       // 2024 MK4iL3 Swerve
  private static final int CHARGEDUP_ROBOT_2023 = 23;       // 2023 MK4iL2 Swerve
  private static final int GHETTOBOT = 99;                  // Mechanum Testbench  
  private static final int ROBOT = CHARGEDUP_ROBOT_2023;    // 2024 Robot 

  public final SendableChooser<Command> autoChooser;

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
    final SubSys_Slider sliderSubSys;
    final SubSys_Climber climberSubSys;

    // Switch Robots
       switch (ROBOT) {
        // ##### CHARGEDUP_ROBOT_2023 #####
        case CHARGEDUP_ROBOT_2023:
        
            // ---- Drive Subsystem ----
            // swerve_ctre
            drivetrain = frc.robot.library.drivetrains.swerve_ctre.mk4il22023.TunerConstants_MK4iL2_2023.DriveTrain;
            
            drive = new SwerveRequest.FieldCentric()
                .withDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd * 0.1)
                .withRotationalDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxRotSpd * 0.1) // Add a 10% deadband
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

            logger = new Telemetry(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd);

            // Auto
            autoChooser = drivetrain.getAutoChooser();

            // ---- Human Machine Interface Station ----
            hmiStation = new HMIStation();

            // Configure the button bindings
            configureButtonBindingsChargedUpRobot2023(drivetrain, drive, logger, hmiStation);
            break;
    
        // ##### GHETTOBOT #####
        case GHETTOBOT:

            // ---- Drive Subsystem ----
            mecanumDriveSubSys = new SubSys_MecanumDrive();

            // Null 
            autoChooser = null;

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
                .withDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd * 0.1)
                .withRotationalDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxRotSpd * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric 
            /*
            drive = new SwerveRequest.RobotCentric()
                .withDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd * 0.1)
                .withRotationalDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxRotSpd * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric    
            */

            logger = new Telemetry(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd);          

            // ---- Human Machine Interface Station ----
            hmiStation = new HMIStation();

            // ---- Intake Subsystem ----
            intakeSubSys = new SubSys_Intake();

            // ---- Shooter Subsystem ----
            shooterSubSys = new SubSys_Shooter();

            // ---- Slider Subsystem ----
            sliderSubSys = new SubSys_Slider();

            // ---- Climber Subsystem ----
            climberSubSys = new SubSys_Climber();
            
            // ---- Auto ----
            // Register Named Commands for PathPlanner
            NamedCommands.registerCommand("IntakePickupNote", new Cmd_SubSys_Intake_PickUpNote());
            NamedCommands.registerCommand("ScoreSpeakerLeft", new Cmd_ScoreSpeakerLeft());
            NamedCommands.registerCommand("ScoreSpeakerRight", new Cmd_ScoreSpeakerRight());
            NamedCommands.registerCommand("ScoreSpeakerCenter", new Cmd_ScoreSpeakerCenter());
            NamedCommands.registerCommand("ScoreAmp", new Cmd_ScoreAmp());
            // Auto Chooser
            autoChooser = drivetrain.getAutoChooser();

            // Configure the button bindings
            configureButtonBindingsCrescendoRobot2024(drivetrain, drive, logger, hmiStation, climberSubSys, sliderSubSys);
            break;
    }
    
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
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
      
        hmiStation.gyroResetButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
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

        intakeSubSys.setDefaultCommand(new Cmd_SubSys_Intake_Default(
            intakeSubSys, 
            0,
            driverStationSubSys::coFwdAxisRaw));

        driverStationSubSys.highConeDelivery.whileTrue(new Cmd_SubSys_Intake_Default(intakeSubSys, 1, null));
        driverStationSubSys.midConeDelivery.whileTrue(new Cmd_SubSys_Intake_Default(intakeSubSys, -1, null));
        
        shooterSubSys.setDefaultCommand(new Cmd_SubSys_Shooter_Default(
            shooterSubSys, 
            0,
            0,
            driverStationSubSys::coFwdAxisRaw));
        
        driverStationSubSys.highSafePos.whileTrue(new Cmd_SubSys_Shooter_Default(shooterSubSys, 0, 1, null));
        driverStationSubSys.groundPickupButton.whileTrue(new Cmd_SubSys_Shooter_Default(shooterSubSys, 0, -1, null));    

        // sliderSubSys.setDefaultCommand(new Cmd_SubSys_Slider_Default());
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
  private void configureButtonBindingsCrescendoRobot2024(
    CommandSwerveDrivetrain drivetrain,
    SwerveRequest.FieldCentric drive,
    Telemetry logger,
    HMIStation hmiStation,
    SubSys_Climber climberSubSys,
    SubSys_Slider sliderSubSys,
    SubSys_Intake subSysIntake,
    SubSys_Shooter subSysShooter) {

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> 
            drive.withVelocityX(hmiStation.driveFwdAxisRaw() * Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd) // Drive forward with negative Y (forward)
              .withVelocityY(hmiStation.driveStrAxisRaw() * Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd) // Drive left with negative X (left)
              .withRotationalRate(hmiStation.driveRotAxisRaw() * Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxRotSpd) // Drive counterclockwise with negative X (left)
        ));

    //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    //joystick.b().whileTrue(drivetrain
    //    .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    hmiStation.gyroResetButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
        drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
    
    hmiStation.climberUp.whileTrue(new climberSetVoltUp(climberSubSys));
    hmiStation.climberDn.whileTrue(new climberSetVoltDn(climberSubSys));
    
    hmiStation.sliderOut.onTrue(new InstantCommand(sliderSubSys::sliderExtendCmd));
    hmiStation.sliderIn.onTrue(new InstantCommand(sliderSubSys::sliderRetractCmd));
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
