package frc.robot.crescendo.subsystems.bling.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.bling.SubSys_Bling;

import static frc.robot.crescendo.subsystems.bling.SubSys_Bling_Constants.Colors.INTAKE_OCCUPIED_COLOR;

public class Cmd_SubSys_Bling_IntakeOccupied extends Command {
    private final SubSys_Bling subSysBling;
    private final Timer timer = new Timer();

    public Cmd_SubSys_Bling_IntakeOccupied(SubSys_Bling subSysBling) {
        this.subSysBling = subSysBling;
    }

    @Override
    public void initialize() {
        subSysBling.setSolidColor(INTAKE_OCCUPIED_COLOR);
        timer.restart();
    }

    @Override
    public void execute() {
        subSysBling.update();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(3);
    }

    @Override
    public boolean runsWhenDisabled() {
        return false; // Set LED's when disabled
    }
}
