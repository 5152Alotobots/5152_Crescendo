// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Arm.Cmd;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain_Constants;

public class Cmd_ArmExtensionPID extends CommandBase {
  /** Creates a new Cmd_SubSyst_DriveTrain_Drive4Distance. */
  private final SubSys_Arm subSys_Arm;

  private double targetPosition;
  private double initialPosition;
  private final PIDController distancePID;

  private final SimpleMotorFeedforward feedForward;

  public Cmd_ArmExtensionPID(SubSys_Arm subSys_Arm, double targetPositionCM) {

    this.subSys_Arm = subSys_Arm;
    this.targetPosition = targetPositionCM;
    this.initialPosition = 0.0;

    this.feedForward =
        new SimpleMotorFeedforward(
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.RotationTrajectoryFF.kS,
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.RotationTrajectoryFF.kV);

    this.distancePID =
        new PIDController(
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Pgain,
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Igain,
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Dgain);
    this.distancePID.setTolerance(
        SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.PositionTolerance,
        SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.VelocityTolerance);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.subSys_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this.subSys_DriveTrain.setPose(new Pose2d());
    double currPose = 0; // CHANGEME
    this.initialPosition = currPose;
    this.distancePID.reset();
    this.distancePID.setSetpoint(this.targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currPose = 0; // CHANGEME

    double mvCmd = this.distancePID.calculate(currPose - this.initialPosition, this.targetPosition);

    SmartDashboard.putNumber("PID", mvCmd);
    this.subSys_Arm.ExtendArm(0, mvCmd);

    SmartDashboard.putNumber("Drive4Distance_xSetpoint", this.distancePID.getSetpoint());
    SmartDashboard.putNumber("Drive4Distance_xError", this.distancePID.getPositionError());
    SmartDashboard.putNumber("Drive4Distance_xCmd", mvCmd);
    SmartDashboard.putBoolean("Drive4Distance_AtSetpoint", this.distancePID.atSetpoint());
    // SmartDashboard.putNumber("Rotate2Heading_Setpoint_Position",
    // this.profiledRotationPID.getSetpoint().position);
    // SmartDashboard.putNumber("Rotate2Heading_SetPoint_Velocity",this.profiledRotationPID.getSetpoint().velocity);
    // SmartDashboard.putNumber("Rotate2Heading_Error",
    // this.profiledRotationPID.getPositionError());

    // SmartDashboard.putNumber("Rotate2Heading_rotPIDCmd", rotPIDCmd);
    // SmartDashboard.putNumber("Rotate2Heading_rotFFCmd", rotFFCmd);
    // SmartDashboard.putNumber("Rotate2Heading_rotCmd", rotCmd);

    /* SETPOINTS */
    // X
    // SmartDashboard.putNumber("Drive4Dist_Setpoint_Position X",
    // this.xDistancePID.getSetpoint().position);
    // SmartDashboard.putNumber("Drive4Dist_SetPoint_Velocity
    // X",this.xDistancePID.getSetpoint().velocity);
    // Y
    // SmartDashboard.putNumber("Drive4Dist_Setpoint_Position Y",
    // this.yDistancePID.getSetpoint().position);
    // SmartDashboard.putNumber("Drive4Dist_SetPoint_Velocity
    // Y",this.yDistancePID.getSetpoint().velocity);
    // ROT
    /* GOALS */
    // X
    // SmartDashboard.putNumber("Drive4Dist_Goal_Position X", this.xDistancePID.getGoal().position);
    // SmartDashboard.putNumber("Drive4Dist_Goal_Velocity X", this.xDistancePID.getGoal().velocity);
    // Y
    // SmartDashboard.putNumber("Drive4Dist_Goal_Position Y", this.yDistancePID.getGoal().position);
    // SmartDashboard.putNumber("Drive4Dist_Goal_Velocity Y", this.yDistancePID.getGoal().velocity);
    // ROT
    /* ERRORS */
    // X
    SmartDashboard.putNumber("Drive4Dist_Velocity Error X", this.distancePID.getVelocityError());
    SmartDashboard.putNumber("Drive4Dist_Error X", this.distancePID.getPositionError());

    /* PIDF GAINS */
    // X
    SmartDashboard.putNumber("Drive4Dist_Pgain X", this.distancePID.getP());
    SmartDashboard.putNumber("Drive4Dist_Igain X", this.distancePID.getI());
    SmartDashboard.putNumber("Drive4Dist_Dgain X", this.distancePID.getD());
    /* Is At Goal */
    // X
    // SmartDashboard.putBoolean("Drive4Dist_IsAtGoal X", this.xDistancePID.atGoal());
    // Y
    // SmartDashboard.putBoolean("Drive4Dist_IsAtGoal Y", this.yDistancePID.atGoal());
    // ROT
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.subSys_Arm.ExtendArm(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /* PID */
    if (this.distancePID.atSetpoint()) {
      return true;
    } else {
      return false;
    }
    /* ProfiledPID
    if (this.xDistancePID.atGoal() &&
         this.yDistancePID.atGoal() &&
         this.profiledRotationPID.atGoal()){
      return true;
    }else{
      return false;
    }
    */
  }
}
