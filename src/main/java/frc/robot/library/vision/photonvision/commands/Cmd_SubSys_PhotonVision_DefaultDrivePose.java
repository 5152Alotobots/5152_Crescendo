// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library.vision.photonvision.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;
import frc.robot.library.vision.photonvision.SubSys_Photonvision;

public class Cmd_SubSys_PhotonVision_DefaultDrivePose extends Command {
    private final SubSys_Photonvision subSysPhotonvision;
    private final CommandSwerveDrivetrain subSysSwerve;

    public Cmd_SubSys_PhotonVision_DefaultDrivePose(
            SubSys_Photonvision subSysPhotonvision,
            CommandSwerveDrivetrain subSysSwerve) {

        this.subSysPhotonvision = subSysPhotonvision;
        this.subSysSwerve = subSysSwerve;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subSysPhotonvision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (subSysPhotonvision.getEstimatedVisionPose2d().isPresent()) {
            Pair<Pose2d, Double> vision = subSysPhotonvision.getEstimatedVisionPose2d().get();
            subSysSwerve.addVisionMeasurement(vision.getFirst(), vision.getSecond());
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
