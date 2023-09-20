/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Library.Vision.Limelight;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Robot;

public class SubSys_LimeLight extends SubsystemBase {
  /** Creates a new LimeLightSubSys. */

  // Coordinate System
  // origin =>	x = length/2
  //				y = width/2
  //				z = top face of swerve units or bottom face of frame
  // +x = front of robot
  // +y = right of robot
  // +z = down

  private double limelight_x_Offset = 0.0;

  private double limelight_y_Offset = 0.0;
  private double limelight_z_Offset = 0.0;
  private double limelight_x_AxisRotationAngOffset = 0.0;
  private double limelight_y_AxixRotationAngOffset = 0.0;
  private double limelight_z_AxixRotationAngOffset = 0.0;
  public double ledMode;
  public double target;

  public double m_Tv;
  public double m_Tx;
  public double m_Ty;
  public double m_Ta;
  public double m_TargetDistance;

  public SubSys_LimeLight() {
    m_TargetDistance = 0.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateLimelightData();
    SmartDashboard.putNumber("TV", m_Tv);
    SmartDashboard.putNumber("TX", m_Tx);
    SmartDashboard.putNumber("TY", m_Ty);
    updateLimelightTargetDistance();
    SmartDashboard.putNumber("TargetDistance", m_TargetDistance);
    SmartDashboard.putNumber("TargetDistance_Feet", Units.metersToFeet(m_TargetDistance));
  }

  public void setLimeLightPosOffsets(
      double limelight_x_Offset,
      double limelight_y_Offset,
      double limelight_z_Offset,
      double limelight_x_AxisRotationAngOffset,
      double limelight_y_AxixRotationAngOffset,
      double limelight_z_AxixRotationAngOffset) {
    this.limelight_x_Offset = limelight_x_Offset;
    this.limelight_y_Offset = limelight_y_Offset;
    this.limelight_z_Offset = limelight_z_Offset;
    this.limelight_x_AxisRotationAngOffset = limelight_x_AxisRotationAngOffset;
    this.limelight_y_AxixRotationAngOffset = limelight_y_AxixRotationAngOffset;
    this.limelight_z_AxixRotationAngOffset = limelight_z_AxixRotationAngOffset;
  }

  public double[] getLimeLightPosOffsets() {
    double[] limelightPosOffsets = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    limelightPosOffsets[0] = this.limelight_x_Offset;
    limelightPosOffsets[1] = this.limelight_y_Offset;
    limelightPosOffsets[2] = this.limelight_z_Offset;
    limelightPosOffsets[3] = this.limelight_x_AxisRotationAngOffset;
    limelightPosOffsets[4] = this.limelight_y_AxixRotationAngOffset;
    limelightPosOffsets[5] = this.limelight_z_AxixRotationAngOffset;

    return limelightPosOffsets;
  }

  // getLimeLightTrgtRelPos
  // 	@trgtID - Limelight Target ID
  //
  // returns the x, y and z distances to the target from the Limelight Camera Position
  public double[] getLimeLightTrgtRelPos(int trgtID) {
    double[] limelightTrgtRelPos = {0.0, 0.0, 0.0};

    return limelightTrgtRelPos;
  }

  // getLimeLightTrgtPos
  // 	@trgtID - Limelight Target ID
  //
  // returns the x, y and z distances to the target from the Robot Origin Position
  public double[] getLimeLightTrgtPos(int trgtID) {
    double[] limelightTrgtPos = {0.0, 0.0, 0.0};

    return limelightTrgtPos;
  }

  // convertLimeLightRelPos2Pos
  // @relPos - (x, y, z) distances
  //
  // returns the x, y and z target distances from the Robot Origin Position from the LimeLight
  // Camera Relative Positions
  public double[] convertLimeLightRelPos2Pos(double[] relPos) {
    double[] Pos = {0.0, 0.0, 0.0};

    Pos[0] = relPos[0] - this.limelight_x_Offset;
    Pos[1] = relPos[1] - this.limelight_y_Offset;
    Pos[2] = relPos[2] - this.limelight_z_Offset;

    return Pos;
  }

  private void updateLimelightData() {
    double lcltv =
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double lcltx =
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double lclty =
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double lclta =
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    SmartDashboard.putNumber("RawTv", lcltv);
    // LinearFilter filtertv = LinearFilter.movingAverage(10);
    // m_Tv = filtertv.calculate(lcltv);
    m_Tv = lcltv;
    SmartDashboard.putNumber("FiltTv", m_Tv);

    SmartDashboard.putNumber("RawTx", lcltx);
    LinearFilter filtertx = LinearFilter.movingAverage(10);
    m_Tx = filtertx.calculate(lcltx);
    SmartDashboard.putNumber("FiltTx", m_Tx);

    SmartDashboard.putNumber("RawTy", lclty);
    // LinearFilter filterty = LinearFilter.movingAverage(10);
    // m_Ty = filterty.calculate(lclty);
    m_Ty = lclty;
    SmartDashboard.putNumber("FiltTy", m_Ty);

    m_Ta = lclta;
  }

  private void updateLimelightTargetDistance() {
    // Rapid React Specific
    double ringDistance = 0.0;
    if (m_Tv > 0.5) {
      ringDistance =
          (Constants.Field.Hub.kTargetRingHeight - Robot.Dimensions.Limelight.kCameraHeight)
              / (Math.tan(Units.degreesToRadians(m_Ty + Robot.Dimensions.Limelight.kCameraAngle)));
      m_TargetDistance = ringDistance + Constants.Field.Hub.kTargetRingDist2Ctr;
    } else {
      m_TargetDistance = 0;
    }
  }
}
