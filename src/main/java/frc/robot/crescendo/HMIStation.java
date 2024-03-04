// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Robot.Calibrations;
import frc.robot.Constants.Robot.Calibrations.DriveTrain;
import frc.robot.library.driverstation.JoystickUtilities;

/**
 * The DriverStation class represents the available inputs to the robot,
 * providing access to controllers
 * and defining buttons for various commands.
 */
 public class HMIStation {
  public static class DriverController {
    private DriverController () {}
    // **** Rate Limits for Drive ****
    static SlewRateLimiter driveXSpdFilter = new SlewRateLimiter(
        DriveTrain.DriveTrainMaxAcceleration,
        DriveTrain.DriveTrainMaxDeceleration, 0);
    static SlewRateLimiter driveYSpdFilter = new SlewRateLimiter(
        DriveTrain.DriveTrainMaxAcceleration,
        DriveTrain.DriveTrainMaxDeceleration, 0);

    static SlewRateLimiter driveSpdPerfModeSwFilter = new SlewRateLimiter(DriveTrain.driveXYSpdPerfModeSwFilterRate);
    static SlewRateLimiter driveRotPerfModeSwFilter = new SlewRateLimiter(DriveTrain.driveRotSpdPerfModeSwFilterRate);

    // **** Driver Controller ****
    private static final XboxController driverController = new XboxController(0);

    public static final JoystickButton shooterRollerIn = new JoystickButton(driverController, 1);
    public static final JoystickButton shooterTransfer = new JoystickButton(driverController, 3);
    public static final JoystickButton gyroResetButton = new JoystickButton(driverController, 4);
    public static final JoystickButton turtleModeButton = new JoystickButton(driverController, 5);
    public static final JoystickButton turboModeButton = new JoystickButton(driverController, 6);

    public static final POVButton climberUp = new POVButton(driverController, 0);
    public static final POVButton climberDn = new POVButton(driverController, 180);
    public static final POVButton shooterSlowButton = new POVButton(driverController, 270);

    /**
     * Gets the forward axis value for driving.
     * 
     * @return The value used for driving forward, with an applied rate limiter
     */
    public static double driveFwdAxisRaw() {
      return driveXSpdFilter.calculate(-1 * driverController.getRawAxis(1));
    }

    /**
     * Gets the strafe axis value for driving.
     *
     * @return The value used for strafeing, with an applied rate limiter
     */
    public static double driveStrAxisRaw() {
      return driveYSpdFilter.calculate(-1 * driverController.getRawAxis(0));
    }

    /**
     * Gets the rotation axis value for driving.
     * 
     * @return The value used for rotation, reversed
     */
    public static double driveRotAxisRaw() {
      return -1 * driverController.getRawAxis(4);
    }

    /**
     * 
     * @return
     */
    public static double getDriveXYPerfMode() {
      double xySpeed = Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd;
      if (turtleModeButton.getAsBoolean()) {
        xySpeed = Calibrations.DriveTrain.PerformanceMode_Turtle.DriveTrainMaxSpd;
      } else if (turboModeButton.getAsBoolean()) {
        xySpeed = Calibrations.DriveTrain.PerformanceMode_Turbo.DriveTrainMaxSpd;
      }
      return driveSpdPerfModeSwFilter.calculate(xySpeed);
    }

    /**
     * 
     * @return
     */
    public static double getDriveRotPerfMode() {
      double rotSpeed = Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxRotSpd;
      if (turtleModeButton.getAsBoolean()) {
        rotSpeed = Calibrations.DriveTrain.PerformanceMode_Turtle.DriveTrainMaxRotSpd;
      } else if (turboModeButton.getAsBoolean()) {
        rotSpeed = Calibrations.DriveTrain.PerformanceMode_Turbo.DriveTrainMaxRotSpd;
      }
      return driveRotPerfModeSwFilter.calculate(rotSpeed);
    }

    /**
     * The trigger for shooting the note into the shooter wheels
     * 
     * @return true if trigger value greater than 0.3
     */
    public static boolean shooterOutTrigger() {
      return (driverController.getRawAxis(3) > 0.3);
    }

    public static final Trigger shooterShoot = new Trigger(() -> shooterOutTrigger());
  }

  public static class CoDriverController {
    private CoDriverController() {}
    // **** Co-Driver Controller ****
    private static final XboxController coDriverController = new XboxController(1);

    // Co-Driver Buttons
    public static final JoystickButton shooterRollerOutSlow = new JoystickButton(coDriverController, 1);
    public static final JoystickButton pickupNoteTransferToShooter = new JoystickButton(coDriverController, 2);
    public static final JoystickButton shooterRollerInSlow = new JoystickButton(coDriverController, 4);
    public static final JoystickButton intakeOut = new JoystickButton(coDriverController, 5);
    public static final JoystickButton intakeIn = new JoystickButton(coDriverController, 6);

    // Co-Driver POV
    public static final POVButton sliderOut = new POVButton(coDriverController, 0);
    public static final POVButton climberSupportExtend = new POVButton(coDriverController, 90);
    public static final POVButton sliderIn = new POVButton(coDriverController, 180);
    public static final POVButton climberSupportRetract = new POVButton(coDriverController, 270);

    // Co-Driver Axes
    /**
     * Gets the forward axis value for driving.
     * 
     * @return The value used for driving forward. unmodified.
     */
    public static double shooterArmAxisRaw() {
      return -1 * coDriverController.getRawAxis(5);
    }

    public static double shooterArmAxis() {
      return JoystickUtilities.joyDeadBndScaled(shooterArmAxisRaw(), .5, 1);
    }

    /**
     * Gets the rotation axis value for driving. unmodified.
     *
     * @return The rotation axis value.
     */
    public static double intakeArmAxisRaw() {
      return coDriverController.getRawAxis(1);
    }

    // Co Driver Trigger Axes
    public static boolean shooterSpeakerPosTrigger() {
      return (coDriverController.getRawAxis(2) > 0.3);
    }

    public static final Trigger shooterSpeakerPos = new Trigger(() -> shooterSpeakerPosTrigger());

    public static boolean shooterAmpPosTrigger() {
      return (coDriverController.getRawAxis(3) > 0.3);
    }

    public static final Trigger shooterAmpPos = new Trigger(() -> shooterAmpPosTrigger());
  }

  public static class AuxDriverController {
    private AuxDriverController() {}
    // Aux Driver Controller
    private static final XboxController auxdriverController = new XboxController(2);
  }

  // ---- Alliance Color
  public void alliancePosition() {
    DriverStation.getAlliance();
  }
}