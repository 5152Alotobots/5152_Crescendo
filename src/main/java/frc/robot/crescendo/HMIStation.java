// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

 /**
 * The DriverStation class represents the available inputs to the robot, providing access to controllers
 * and defining buttons for various commands.
 */
public class HMIStation {

    // Driver Controller
    private final XboxController driverController = new XboxController(0);

    public final JoystickButton gyroResetButton = new JoystickButton(driverController, 4);
 
    public final POVButton climberUp = new POVButton(driverController, 0);
    public final POVButton climberDn = new POVButton(driverController, 180);
    
    public final JoystickButton testButton = new JoystickButton(driverController, 3);

    // CoDriver Controller
    private final XboxController coDriverController = new XboxController(1);

     public final JoystickButton shooterIn = new JoystickButton(coDriverController, 4);
     public final JoystickButton shooterOut = new JoystickButton(coDriverController, 1);
     public final JoystickButton shooterShoot = new JoystickButton(coDriverController, 3);

    public final JoystickButton intakeIn = new JoystickButton(coDriverController, 5);
    public final JoystickButton intakeOut = new JoystickButton(coDriverController, 6);

    public final POVButton resetLEDColorButton = new POVButton(coDriverController, 180);
    public final POVButton rainbowLEDColorButton = new POVButton(coDriverController, 0);
    public final POVButton rainbowStrobeLEDColorButton = new POVButton(coDriverController, 270);

    private final XboxController auxdriverController = new XboxController(2);
    
     public final JoystickButton sliderIn = new JoystickButton(auxdriverController, 2);
     public final JoystickButton sliderOut = new JoystickButton(auxdriverController, 4);

  /**
     * Gets the alliance
     * @return The Current Alliance
     */
  public void alliancePosition() {
    DriverStation.getAlliance();
  }

  /**
     * Gets the forward axis value for driving.
     * @return The value used for driving forward. unmodified. 
     */
  public double driveFwdAxisRaw() {
      return driverController.getRawAxis(1);
  }

  /**
     * Gets the strafe axis value for driving. unmodified.
     * @return The strafe axis value.
     */
  public double driveStrAxisRaw() {
    return driverController.getRawAxis(0);
  }

  /**
   * Gets the rotation axis value for driving. unmodified.
   * @return The rotation axis value.
   */
  public double driveRotAxisRaw() {
    return driverController.getRawAxis(4);
  }

  /**
   * Gets the axis value for rotating the Shooter.
   * @return The value used for driving forward. unmodified. 
   */
  public double shooterRotateAxisRaw() {
    return coDriverController.getRawAxis(5);
  }

  /**
   * Gets the strafe axis value for driving. unmodified.
   * @return The strafe axis value.
   */
  public double intakeArmAxisRaw() {
    return coDriverController.getRawAxis(1);
  }

  /**
   * Checks if the left rotation point button is pressed.
   * @return True if the button is pressed, false otherwise.
   */
  public boolean rotateLeftPt() {
    return driverController.getRawButton(5);
  }

  /**
   * Checks if the right rotation point button is pressed.
   * @return True if the button is pressed, false otherwise.
   */
  public boolean rotateRightPt() {
    return driverController.getRawButton(6);
  }

  /**
   * Checks if the performance mode A is active.
   * @return True if the mode is active, false otherwise.
   */
  public boolean drivePerfModeAActive() {
    return (driverController.getRawAxis(2) > 0.3);
  }

  /**
   * Checks if the performance mode B is active.
   * @return True if the mode is active, false otherwise.
   */
  public boolean drivePerfModeBActive() {
    return (driverController.getRawAxis(3) > 0.3);
  }

  public static final class Constants {
    public static final double DRIVER_XY_DEADBAND = 0.1;
    public static final double DRIVER_ROT_DEADBAND = 0.1;

  }

}

