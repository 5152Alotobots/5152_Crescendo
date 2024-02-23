// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.Robot.Calibrations;
import frc.robot.Constants.Robot.Calibrations.DriveTrain.PerformanceMode_Default;

 /**
 * The DriverStation class represents the available inputs to the robot, providing access to controllers
 * and defining buttons for various commands.
 */
public class HMIStation {

  SlewRateLimiter driveSpdFilter = new SlewRateLimiter(PerformanceMode_Default.DriveTrainMaxAccel);
  SlewRateLimiter driveRotFilter = new SlewRateLimiter(PerformanceMode_Default.DriveTrainMaxRotAccel);

  // **** Driver Controller ****
  private final XboxController driverController = new XboxController(0);

  // Driver Buttons
  public final JoystickButton shooterRollerIn = new JoystickButton(driverController, 1);
  //public final JoystickButton blankDriverBButton = new JoystickButton(driverController, 2);
  //public final JoystickButton blankDriverXButton = new JoystickButton(driverController, 3);
  public final JoystickButton gyroResetButton = new JoystickButton(driverController, 4);
  public final JoystickButton turtleModeButton = new JoystickButton(driverController, 5);
  public final JoystickButton turboModeButton = new JoystickButton(driverController, 6);

  // CoDriver Controller
  private final XboxController coDriverController = new XboxController(1);

  // public final JoystickButton shooterIn = new JoystickButton(coDriverController, 4);
  public final JoystickButton shooterOut = new JoystickButton(coDriverController, 1);
  public final JoystickButton shooterShoot = new JoystickButton(coDriverController, 3);
  public final JoystickButton shooterRollerOutSlow = new JoystickButton(coDriverController, 1);
  // public final JoystickButton shooterSpeakerPos = new JoystickButton(coDriverController, 2);
  // public final JoystickButton shooterAmpPos = new JoystickButton(coDriverController, 3);
  public final JoystickButton shooterRollerInSlow = new JoystickButton(coDriverController, 4);

  public final JoystickButton intakeIn = new JoystickButton(coDriverController, 5);
  public final JoystickButton intakeOut = new JoystickButton(coDriverController, 6);

  public final POVButton climberUp = new POVButton(driverController, 0);
  public final POVButton climberDn = new POVButton(driverController, 180);
  public final POVButton rainbowStrobeLEDColorButton = new POVButton(coDriverController, 270);

  private final XboxController auxdriverController = new XboxController(2);
    
  public final JoystickButton sliderIn = new JoystickButton(auxdriverController, 2);
  public final JoystickButton sliderOut = new JoystickButton(auxdriverController, 4);

  // Driver Axes
  /**
   * Gets the forward axis value for driving.
   * 
   * @return The value used for driving forward. unmodified. 
   */
  public double driveFwdAxisRaw() {
    return -1*driverController.getRawAxis(1);
  }

  /**
   * Gets the strafe axis value for driving. unmodified.
   *
   * @return The strafe axis value.
   */ 
  public double driveStrAxisRaw() {
    return -1*driverController.getRawAxis(0);
  }

  /**
   * Gets the rotation axis value for driving. unmodified.
   * @return The rotation axis value.
   */ 
  public double driveRotAxisRaw() {
    return -1*driverController.getRawAxis(4);
  }
  
  // Driver Trigger Axes
  /**
   * Checks if the performance mode A is active.
   *
   * @return True if the mode is active, false otherwise.
   */
  public boolean shooterIn() {
    return (driverController.getRawAxis(2) > 0.3);
  }

  /**
   * Checks if the performance mode B is active.
   *
   * @return True if the mode is active, false otherwise.
   */
  public boolean shooterOut() {
    return (driverController.getRawAxis(3) > 0.3);
  }

  /**
   * Gets the forward axis value for driving.
   * 
   * @return The value used for driving forward. unmodified. 
   */
  public double shooterArmAxisRaw() {
    return coDriverController.getRawAxis(1);
  }
  /**
   * Gets the rotation axis value for driving. unmodified.
   *
   * @return The rotation axis value.
   */ 
  public double intakeArmAxisRaw() {
    return coDriverController.getRawAxis(5);
  }
  // Driver Trigger Axes
  /**
   * Checks if the performance mode A is active.
   * @return True if the mode is active, false otherwise.
   */
  public boolean shooterInCoDrvr() {
    return (coDriverController.getRawAxis(3) > 0.3);
  }

  /**
   * Checks if the performance mode B is active.
   * @return True if the mode is active, false otherwise.
   */
  public boolean shooterSourcePos() {
    return (driverController.getRawAxis(2) > 0.3);
  }
  
  // Aux Driver Controller
  //private final XboxController auxdriverController = new XboxController(2);

  // Button Box

  /**
     * 
     * @return
     */
    public double getDriveXYPerfMode(){
        double xySpeed = Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd;
        if(turtleModeButton.getAsBoolean()){
            xySpeed = Calibrations.DriveTrain.PerformanceMode_Turtle.DriveTrainMaxSpd;
        }else if(turboModeButton.getAsBoolean()){
            xySpeed = Calibrations.DriveTrain.PerformanceMode_Turbo.DriveTrainMaxSpd;
        }
        return driveSpdFilter.calculate(xySpeed);
    }

     /**
     * 
     * @return
     */
    public double getDriveRotPerfMode(){
        double rotSpeed = Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxRotSpd;
        if(turtleModeButton.getAsBoolean()){
            rotSpeed = Calibrations.DriveTrain.PerformanceMode_Turtle.DriveTrainMaxRotSpd;
        }else if(turboModeButton.getAsBoolean()){
            rotSpeed = Calibrations.DriveTrain.PerformanceMode_Turbo.DriveTrainMaxRotSpd;
        }
        return driveRotFilter.calculate(rotSpeed);
    }
  
  // ---- Alliance Color
  public void alliancePosition(){
    DriverStation.getAlliance();
  }
}
