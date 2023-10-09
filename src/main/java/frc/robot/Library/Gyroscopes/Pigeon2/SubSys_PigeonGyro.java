/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Library.Gyroscopes.Pigeon2;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Library.Gyroscopes.NavX.SubSys_NavXGyro_Constants;

public class SubSys_PigeonGyro extends SubsystemBase {
  /** Creates a new NavXGyro. */
  private Pigeon2 pigeon2Gyro;

  private double gyroOffsetDegrees;

  public SubSys_PigeonGyro() {
    this.pigeon2Gyro = new Pigeon2(Constants.CAN_IDs.Pigeon2_ID);
    this.gyroOffsetDegrees = 0.0;
    // m_Pigeon2Gyro.setYaw(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Display
    SmartDashboard.putNumber("Pigeon_getRawYaw", getRawYaw());
    SmartDashboard.putNumber("Pigeon_getYaw", getYaw());
    SmartDashboard.putNumber("Pigeon_getYawRotation2d", getYawRotation2d().getDegrees());
    // SmartDashboard.putNumber("GyroCompass", m_Pigeon2Gyro.getCompassHeading());
  }

  public double getRawGyroPitch() {
    return this.pigeon2Gyro.getPitch();
  }

  public double getRawGyroRoll() {
    return this.pigeon2Gyro.getRoll();
  }
  /**
   * Return Raw Gyro Angle
   *
   * @return Raw Angle double raw Gyro Angle
   */
  public double getRawYaw() {
    return this.pigeon2Gyro.getYaw();
  }

  public double getYaw() {
    return getRawYaw() - this.gyroOffsetDegrees;
  }

  /**
   * Return Gyro Angle in Degrees
   *
   * @return Gyro Angle double Degrees
   */
  public double getYawAngle() {
    return Math.IEEEremainder(getYaw(), 360)
        * (SubSys_NavXGyro_Constants.GyroReversed ? -1.0 : 1.0);
  }

  /**
   * Return Gyro Angle in Rotation2d
   *
   * @return Gyro Angle Rotation2d
   */
  public Rotation2d getYawRotation2d() {
    return Rotation2d.fromDegrees(getYawAngle());
  }

  /** Zero Gyro */
  public void zeroYaw() {
    this.gyroOffsetDegrees = getRawYaw();
  }

  /** Set Gyro */
  public void setYaw(double yawDegrees) {
    this.gyroOffsetDegrees = getRawYaw() - yawDegrees;
  }
}
