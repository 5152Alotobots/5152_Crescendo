// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.ColorSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.ChargedUp.ColorSensor.Const_ColorSensor;

public class SubSys_ColorSensor extends SubsystemBase {
  public SubSys_ColorSensor() {}

  // private final I2C.Port i2cPort = I2C.Port.kOnboard;
  // private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  // private Color detectedColor;
  // private double amountOfBlue = detectedColor.blue;

  // private double IR;

  @Override
  public void periodic() {

    // detectedColor = m_colorSensor.getColor();

    // IR = m_colorSensor.getIR();

    // SmartDashboard.putNumber("Red", detectedColor.red);
    // SmartDashboard.putNumber("Green", detectedColor.green);
    // SmartDashboard.putNumber("Blue", detectedColor.blue);
    // SmartDashboard.putNumber("IR", IR);
    // SmartDashboard.putString("Game Obj ColSens", GetTypeOfGameElement().toString());
  }

  public String GetTypeOfGameElement() {
    // @return GameElement can be CONE, CUBE, or NA
    // if (amountOfBlue < Const_ColorSensor.kMAXAmountOfBlueInCONE) return "CONE";
    // if (amountOfBlue > Const_ColorSensor.kMINAmountOfBlueInCUBE) return "CUBE";

    return "N/A";
  }
}
