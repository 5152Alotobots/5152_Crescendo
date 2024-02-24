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
import frc.robot.Constants.Robot.Calibrations.DriveTrain.PerformanceMode_Default;

 /**
 * The DriverStation class represents the available inputs to the robot, providing access to controllers
 * and defining buttons for various commands.
 */
public class HMIStation {
  
  SlewRateLimiter driveXSpdFilter = new SlewRateLimiter(DriveTrain.DriveTrainMaxAccel,DriveTrain.DriveTrainMaxDeccel,0);
  SlewRateLimiter driveYSpdFilter = new SlewRateLimiter(DriveTrain.DriveTrainMaxAccel,DriveTrain.DriveTrainMaxDeccel,0);

  SlewRateLimiter driveSpdPerfModeSwFilter = new SlewRateLimiter(DriveTrain.driveXYSpdPerfModeSwFilterRate);
  SlewRateLimiter driveRotPerfModeSwFilter = new SlewRateLimiter(DriveTrain.driveRotSpdPerfModeSwFilterRate);

  // **** Driver Controller ****
  private final XboxController driverController = new XboxController(0);

  // Driver Buttons
  public final JoystickButton shooterRollerIn = new JoystickButton(driverController, 1);
  public final JoystickButton intakePickupNote = new JoystickButton(driverController, 2);
  public final JoystickButton shooterTransfer = new JoystickButton(driverController, 3);
  public final JoystickButton gyroResetButton = new JoystickButton(driverController, 4);
  public final JoystickButton turtleModeButton = new JoystickButton(driverController, 5);
  public final JoystickButton turboModeButton = new JoystickButton(driverController, 6);

  // Driver POV
  public final POVButton climberUp = new POVButton(driverController, 0);
  //public final POVButton blankDriverPOVRightButton = new POVButton(driverController, 90);
  public final POVButton climberDn = new POVButton(driverController, 180);
  public final POVButton shooterSlowButton = new POVButton(driverController, 270);

  // Driver Axes
   public double driveFwdAxisRaw() {
    //return -1*driverController.getRawAxis(1);
    return driveXSpdFilter.calculate(-1*driverController.getRawAxis(1));
  }

  public double driveStrAxisRaw() {
    //return -1*driverController.getRawAxis(0);
    return driveYSpdFilter.calculate(-1*driverController.getRawAxis(0));
  }

   public double driveRotAxisRaw() {
    return -1*driverController.getRawAxis(4);
  }
  
  // Driver Trigger Axes
  public boolean shooterInTriggerAxis() {
    return (driverController.getRawAxis(2) > 0.3);
  }
  public final Trigger shooterIn = new Trigger(() -> shooterInTriggerAxis());

  public boolean shooterOutTriggerAxis() {
    return (driverController.getRawAxis(3) > 0.3);
  }
  public final Trigger shooterOut = new Trigger(() -> shooterOutTriggerAxis());
  
  // **** Co-Driver Controller ****
  private final XboxController coDriverController = new XboxController(1);

  // Co-Driver Buttons
  public final JoystickButton shooterRollerOutSlow = new JoystickButton(coDriverController, 1);
  public final JoystickButton shooterSpeakerPos = new JoystickButton(coDriverController, 2);
  //public final JoystickButton shooterAmpPos = new JoystickButton(coDriverController, 3);
  public final JoystickButton shooterRollerInSlow = new JoystickButton(coDriverController, 4);
  public final JoystickButton intakeOut = new JoystickButton(coDriverController, 5);
  public final JoystickButton intakeIn = new JoystickButton(coDriverController, 6);

  // Co-Driver POV
  public final POVButton sliderOut = new POVButton(coDriverController, 0);
  //public final POVButton blankDriverPOVRightButton = new POVButton(coDriverController, 90);
  public final POVButton sliderIn = new POVButton(coDriverController, 180);
  //public final POVButton blankCoDriverPOVLeftButton = new POVButton(coDriverController, 270);

  // Co-Driver Axes
  public double shooterArmAxisRaw() {
    return -1*coDriverController.getRawAxis(5);
  }

  public double intakeArmAxisRaw() {
    return coDriverController.getRawAxis(1);
  }

  public boolean shooterInCoDrvrTriggerAxis() {
    return (coDriverController.getRawAxis(3) > 0.3);
  }
  public final Trigger shooterInCoDrvr = new Trigger(() -> shooterInCoDrvrTriggerAxis());

  public boolean shooterArmSourcePosTriggerAxis() {
    return (driverController.getRawAxis(2) > 0.3);
  }
  public final Trigger shooterArmSourcePos = new Trigger(() -> shooterArmSourcePosTriggerAxis());
  
  // Aux Driver Controller
  //private final XboxController auxdriverController = new XboxController(2);

  // Button Box

 
  public double getDriveXYPerfMode(){
    double xySpeed = Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd;
    if(turtleModeButton.getAsBoolean()){
      xySpeed = Calibrations.DriveTrain.PerformanceMode_Turtle.DriveTrainMaxSpd;
    }else if(turboModeButton.getAsBoolean()){
      xySpeed = Calibrations.DriveTrain.PerformanceMode_Turbo.DriveTrainMaxSpd;
    }
      return driveSpdPerfModeSwFilter.calculate(xySpeed);
  }

  public double getDriveRotPerfMode(){
    double rotSpeed = Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxRotSpd;
    if(turtleModeButton.getAsBoolean()){
      rotSpeed = Calibrations.DriveTrain.PerformanceMode_Turtle.DriveTrainMaxRotSpd;
    }else if(turboModeButton.getAsBoolean()){
      rotSpeed = Calibrations.DriveTrain.PerformanceMode_Turbo.DriveTrainMaxRotSpd;
    }
      return driveRotPerfModeSwFilter.calculate(rotSpeed);
  }
  
  // ---- Alliance Color
  public void alliancePosition(){
    DriverStation.getAlliance();
  }

}