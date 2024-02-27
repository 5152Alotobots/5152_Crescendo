package frc.robot.library.vision.photonvision.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.library.vision.photonvision.SubSys_Photonvision;

public class Cmd_SubSys_Photonvision_Default extends Command {
    private SubSys_Photonvision subSysPhotonvision;

    public Cmd_SubSys_Photonvision_Default(SubSys_Photonvision subSysPhotonvision) {
        this.subSysPhotonvision = subSysPhotonvision;
        addRequirements(subSysPhotonvision);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (subSysPhotonvision.getEstimatedVisionPose2d().isPresent()) {
            SmartDashboard.putString("Vision/Vision Pose Estimate", String.valueOf(subSysPhotonvision.getEstimatedVisionPose2d().get().getFirst()));
        } else {
            SmartDashboard.putString("Vision/Vision Pose Estimate", "NONE");
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
