package frc.robot.crescendo.subsystems.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;

import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Intake_ArmPosition extends Command {
    SubSys_Intake subSysIntake;
    double posCmd;
    Timer timer = new Timer();

    /**
     * Rotates the intake to a degree offset from (down position) 0 degrees
     */
    public Cmd_SubSys_Intake_ArmPosition(SubSys_Intake subSysIntake, double posCmd) {
        this.subSysIntake = subSysIntake;
        this.posCmd = posCmd;
    }

    @Override
    public void initialize() {
        subSysIntake.setIntakeArmVelVoltsMMPos(posCmd);
    }

    @Override
    public void execute() {
        subSysIntake.setIntakeArmVelVoltsMMPos(posCmd);
    
        double armPosError = posCmd - subSysIntake.getIntakeArmPosition();
        boolean armVelError = subSysIntake.getIntakeArmVelocity() < 5;
    boolean rightWheelsVelAtSpd = rightWheelsVelError < 5;
    boolean leftWheelsStable = subSysShooter.getShooterLeftWheelsAcceleration() < 10;
    boolean rightWheelsStable = subSysShooter.getShooterRightWheelsAcceleration() < 10;

    if(leftWheelsVelAtSpd && rightWheelsVelAtSpd && leftWheelsStable && rightWheelsStable){
      atSpd = true;
    }else{
      atSpd = false;
    }

    if(atSpd && atPos){
      readyToShoot = true;
    }
        
    SmartDashboard.putBoolean("ShooterAimSpinUp_atPos", atPos);
    SmartDashboard.putBoolean("Shooter_AimSpinUp_atSpd", atSpd);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        subSysIntake.setIntakeArmSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return subSysIntake.intakeArmMtrAtSetpoint() && timer.get() > 0.8; // account for delay
    }
}
