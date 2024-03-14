package frc.robot.crescendo.subsystems.bling.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.bling.SubSys_Bling;

import static frc.robot.crescendo.subsystems.bling.SubSys_Bling_Constants.Colors.INTAKE_OCCUPIED_COLOR;

public class Cmd_SubSys_Bling_IntakeOccupied extends Command {
    private final SubSys_Bling subSysBling;

    public Cmd_SubSys_Bling_IntakeOccupied(SubSys_Bling subSysBling) {
        this.subSysBling = subSysBling;
        addRequirements(subSysBling);
    }

    @Override
    public void initialize() {
        subSysBling.setSolidColor(INTAKE_OCCUPIED_COLOR);
    }

    @Override
    public void execute() {
        subSysBling.update();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false; // Set LED's when disabled
    }
}
