package frc.robot.ChargedUp.Arm.Cmd;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ChargedUp.Arm.Const_Arm;
import frc.robot.ChargedUp.Arm.SubSys_Arm;

public class Cmd_ArmShoulderPID extends CommandBase {
  /** Creates a new Cmd_SubSys_DriveTrain_Rotate2Heading. */
  private final SubSys_Arm subSys_Arm;

  private final double targetHeadingDegrees;
  private final SimpleMotorFeedforward feedForward;
  private final ProfiledPIDController profiledRotationPID;
  private final TrapezoidProfile.Constraints profiledRotationConstraints;

  public Cmd_ArmShoulderPID(SubSys_Arm subSys_Arm, double targetHeadingDegrees) {

    this.subSys_Arm = subSys_Arm;
    this.targetHeadingDegrees = targetHeadingDegrees;

    this.profiledRotationConstraints =
        new TrapezoidProfile.Constraints(
            Units.radiansToDegrees(Const_Arm.Trajectory.kShoulderMaxRotSpeed),
            Units.radiansToDegrees(Const_Arm.Trajectory.kShoulderMaxRotAcelSpeed));

    this.feedForward = new SimpleMotorFeedforward(Const_Arm.FF.kS, Const_Arm.FF.kV);

    this.profiledRotationPID =
        new ProfiledPIDController(
            Const_Arm.PID.kP, Const_Arm.PID.kI, Const_Arm.PID.kD, this.profiledRotationConstraints);

    this.profiledRotationPID.enableContinuousInput(-180, 180);
    this.profiledRotationPID.setTolerance(2, 4);
    this.profiledRotationPID.setIntegratorRange(-.3, 0.3);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.subSys_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.profiledRotationPID.reset(this.subSys_Arm.getShoulderRotation());
    this.profiledRotationPID.setGoal(this.targetHeadingDegrees);

    // log target to dashboard
    SmartDashboard.putData(profiledRotationPID);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rotFFCmd = this.feedForward.calculate(this.profiledRotationPID.getSetpoint().velocity);
    double rotPIDCmd =
        this.profiledRotationPID.calculate(
            this.subSys_Arm.getShoulderRotation(), this.targetHeadingDegrees);
    double rotCmd = rotFFCmd + rotPIDCmd;

    // rotate arm
    this.subSys_Arm.RotateArm(0, rotCmd);

    SmartDashboard.putNumber("Rotate2Heading_Goal", this.profiledRotationPID.getGoal().position);
    SmartDashboard.putNumber(
        "Rotate2Heading_Setpoint_Position", this.profiledRotationPID.getSetpoint().position);
    SmartDashboard.putNumber(
        "Rotate2Heading_SetPoint_Velocity", this.profiledRotationPID.getSetpoint().velocity);
    SmartDashboard.putNumber("Rotate2Heading_Error", this.profiledRotationPID.getPositionError());

    SmartDashboard.putNumber("Rotate2Heading_rotPIDCmd", rotPIDCmd);
    SmartDashboard.putNumber("Rotate2Heading_rotFFCmd", rotFFCmd);
    SmartDashboard.putNumber("Rotate2Heading_rotCmd", rotCmd);

    // Test to see if finished
    SmartDashboard.putBoolean("Rotate2Heading Finished", this.profiledRotationPID.atGoal());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.subSys_Arm.RotateArm(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (Math.abs(this.targetHeadingDegrees-this.subSys_DriveTrain.getHeading().getDegrees())< 1){
    //  return true;
    // }else{
    //  return false;
    // }

    if (this.profiledRotationPID.atGoal()) {
      return true;
    } else {
      return false;
    }
  }
}
