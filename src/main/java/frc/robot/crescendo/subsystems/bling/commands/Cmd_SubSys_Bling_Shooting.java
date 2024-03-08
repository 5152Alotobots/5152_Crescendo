package frc.robot.crescendo.subsystems.bling.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.bling.SubSys_Bling;

import static frc.robot.crescendo.subsystems.bling.SubSys_Bling_Constants.Animations.SHOOTING_ANIMATION;

public class Cmd_SubSys_Bling_Shooting extends Command {
    private final SubSys_Bling subSysBling;

    public Cmd_SubSys_Bling_Shooting(SubSys_Bling subSysBling) {
        this.subSysBling = subSysBling;
    }

    @Override
    public void initialize() {
        subSysBling.clearAll();
        subSysBling.runAnimation(SHOOTING_ANIMATION);
    }

    @Override
    public void execute() {
        subSysBling.update();
    }

    @Override
    public void end(boolean interrupted) {
        subSysBling.runDefault();
    }

    @Override
    public boolean isFinished() {
        return false; // Should be interrupted when shooter changes
    }

    @Override
    public boolean runsWhenDisabled() {
        return true; // Set LED's when disabled
    }
}
