package frc.robot.crescendo.subsystems.bling.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.bling.SubSys_Bling;

public class Cmd_SubSys_Bling_DefaultSetToAllianceColor extends Command {
    private final SubSys_Bling subSysBling;

    public Cmd_SubSys_Bling_DefaultSetToAllianceColor(SubSys_Bling subSysBling) {
        this.subSysBling = subSysBling;
        addRequirements(subSysBling);
    }

    @Override
    public void initialize() {
        subSysBling.setLedToAllianceColor();
    }

    @Override
    public void execute() {
        subSysBling.update();
    }

    @Override
    public void end(boolean interrupted) {
        // Nothing as this is default
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true; // Set LED's when disabled
    }
}
