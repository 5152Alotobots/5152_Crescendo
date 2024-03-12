// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.AutoAim;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.PresentArmPositions;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.ShooterWheels;
import frc.robot.crescendo.subsystems.shooter.util.ShooterIntakeDirection;

public class Cmd_SubSys_Shooter_AimSpinUp extends Command {
  /** Creates a new Cmd_SubSys_Shooter_ShootSpeaker. */
  SubSys_Shooter subSysShooter;
  double shooterArmPosCmd;
  double shooterWheelsSpdCmd;
  boolean atPos = false;
  boolean atSpd = false;
  boolean readyToShoot = false;
 
  public Cmd_SubSys_Shooter_AimSpinUp(
    double shooterArmPosCmd,
    double shooterWheelsSpdCmd,
    SubSys_Shooter subSysShooter) {

    this.shooterArmPosCmd = shooterArmPosCmd;
    this.shooterWheelsSpdCmd = shooterWheelsSpdCmd;
    this.subSysShooter = subSysShooter;
    addRequirements(subSysShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subSysShooter.setShooterArmDegree(shooterArmPosCmd);
    atPos = false;

    subSysShooter.setShooterWheelsVelocity(shooterWheelsSpdCmd);
    atSpd = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    atPos = subSysShooter.shooterArmMtrAtSetpoint();
    atSpd = subSysShooter.setShooterWheelsVelocity(shooterWheelsSpdCmd);
    
    if (!readyToShoot){
      if(atPos && atSpd){
        readyToShoot = true;
      }
    }
    SmartDashboard.putBoolean("ShooterAimSpinUp_atPos", atPos);
    SmartDashboard.putBoolean("Shooter_AimSpinUp_atSpd", atSpd);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return readyToShoot;
  }
}
