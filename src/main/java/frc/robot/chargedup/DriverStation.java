// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * The DriverStation class represents the available inputs to the robot, providing access to controllers
 * and defining buttons for various commands.
 */
public final class DriverStation {
  private final XboxController driverController = new XboxController(0);

  public final JoystickButton gyroResetButton = new JoystickButton(driverController, 4);
  public final JoystickButton closeHandButton = new JoystickButton(driverController, 6);
  public final JoystickButton openHandButton = new JoystickButton(driverController, 5);
  public final JoystickButton poseResetButton = new JoystickButton(driverController, 1);
  public final JoystickButton testButton = new JoystickButton(driverController, 3);

  private final XboxController coDriverController = new XboxController(1);

  public final JoystickButton groundPickupButton = new JoystickButton(coDriverController, 1);
  public final JoystickButton highConeDelivery = new JoystickButton(coDriverController, 4);
  public final JoystickButton midConeDelivery = new JoystickButton(coDriverController, 3);
  public final JoystickButton highSafePos = new JoystickButton(coDriverController, 2);
  public final JoystickButton requestConeButton = new JoystickButton(coDriverController, 6);
  public final JoystickButton requestCubeButton = new JoystickButton(coDriverController, 5);
  public final POVButton resetLEDColorButton = new POVButton(coDriverController, 180);
  public final POVButton rainbowLEDColorButton = new POVButton(coDriverController, 0);
  public final POVButton rainbowStrobeLEDColorButton = new POVButton(coDriverController, 270);

  private final XboxController auxdriverController = new XboxController(2);

  /**
   * Gets the forward axis value for driving.
   * 
   * @return The value used for driving forward. unmodified. 
   */
  public double driveFwdAxisRaw() {
    return driverController.getRawAxis(1);
  }

  /**
   * Gets the strafe axis value for driving. unmodified.
   *
   * @return The strafe axis value.
   */
  public double driveStrAxisRaw() {
    return driverController.getRawAxis(0);
  }

  /**
   * Gets the rotation axis value for driving. unmodified.
   *
   * @return The rotation axis value.
   */
  public double driveRotAxisRaw() {
    return driverController.getRawAxis(4);
  }

    /**
   * Gets the forward axis value for driving.
   * 
   * @return The value used for driving forward. unmodified. 
   */
  public double coFwdAxisRaw() {
    return coDriverController.getRawAxis(1);
  }

  /**
   * Gets the strafe axis value for driving. unmodified.
   *
   * @return The strafe axis value.
   */
  public double coStrAxisRaw() {
    return coDriverController.getRawAxis(0);
  }

  /**
   * Gets the rotation axis value for driving. unmodified.
   *
   * @return The rotation axis value.
   */
  public double coRotAxisRaw() {
    return coDriverController.getRawAxis(4);
  }


  /**
   * Checks if the left rotation point button is pressed.
   *
   * @return True if the button is pressed, false otherwise.
   */
  public boolean rotateLeftPt() {
    return driverController.getRawButton(5);
  }

  /**
   * Checks if the right rotation point button is pressed.
   *
   * @return True if the button is pressed, false otherwise.
   */
  public boolean rotateRightPt() {
    return driverController.getRawButton(6);
  }

  /**
   * Checks if the performance mode A is active.
   *
   * @return True if the mode is active, false otherwise.
   */
  public boolean drivePerfModeAActive() {
    return (driverController.getRawAxis(2) > 0.3);
  }

  /**
   * Checks if the performance mode B is active.
   *
   * @return True if the mode is active, false otherwise.
   */
  public boolean drivePerfModeBActive() {
    return (driverController.getRawAxis(3) > 0.3);
  }

  // ----- Arm Subsystem

  /**
   * Gets the shoulder rotate axis value for the arm subsystem. unmodified.
   *
   * @return The shoulder rotate axis value.
   */
  public double getArmRotateAxis() {
    return coDriverController.getRawAxis(1);
  }

  /**
   *
   * @return The arm extend axis value.
   */
  public double getArmExtendAxis() {
    return coDriverController.getRawAxis(5);
  }
}