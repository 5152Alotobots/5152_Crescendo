// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.DriverStation;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class SubSys_DriverStation extends Subsystem {
  /** Creates a new DriverStationSubSys. */

  // Driver Controller
  private XboxController m_DriverController = new XboxController(0);

  public JoystickButton GyroResetButton = new JoystickButton(m_DriverController, 4);
  public JoystickButton CloseHandButton = new JoystickButton(m_DriverController, 6);
  public JoystickButton OpenHandButton = new JoystickButton(m_DriverController, 5);
  public JoystickButton PoseResetButton = new JoystickButton(m_DriverController, 1);
  // public JoystickButton TestButton = new JoystickButton(m_DriverController, 3);

  // Co-Driver Controller
  private XboxController m_CoDriverController = new XboxController(1);
  //Define Joystick Buttons here 

  // AuxDriver Controller
  private XboxController m_AuxDriverController = new XboxController(2);

  public SubSys_DriverStation() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // ---- Drive Subsystem
  // Drive Fwd Axis
  public double DriveFwdAxis() {
    return -m_DriverController.getRawAxis(1);
  }

  // Drive Strafe Axis
  public double DriveStrAxis() {
    return -m_DriverController.getRawAxis(0);
  }

  // Drive Rotate Axis
  public double DriveRotAxis() {
    return -m_DriverController.getRawAxis(4);
  }

  // Drive RotateLeftPoint
  public boolean RotateLeftPt() {
    return m_DriverController.getRawButton(5);
  }

  // Drive RotateRightPoint
  public boolean RotateRightPt() {
    return m_DriverController.getRawButton(6);
  }

  // Drive Performance Mode A
  public boolean DrivePerfModeAActive() {
    return (m_DriverController.getRawAxis(2) > 0.3);
  }

  // Drive Performance Mode B
  public boolean DrivePerfModeBActive() {
    return (m_DriverController.getRawAxis(3) > 0.3);
  }

}
