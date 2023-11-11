package frc.robot.Library.DriveTrains.SwerveDrive.Commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ConstantsSwerve.Auton;
import frc.robot.Library.DriveTrains.SwerveDrive.SubSys_Swerve;

public class FollowTrajectory extends SequentialCommandGroup
{

  public FollowTrajectory(SubSys_Swerve drivebase, PathPlannerTrajectory trajectory, boolean resetOdometry)
  {
    addRequirements(drivebase);

    if (resetOdometry)
    {
      drivebase.resetOdometry(trajectory.getInitialHolonomicPose());
    }

    addCommands(
        new PPSwerveControllerCommand(
            trajectory,
            drivebase::getPose,
            Auton.xAutoPID.createPIDController(),
            Auton.yAutoPID.createPIDController(),
            Auton.angleAutoPID.createPIDController(),
            drivebase::setChassisSpeeds,
            drivebase)
               );
  }
}
