// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.AutoAim;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.PresentArmPositions;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.ShooterWheels;
import frc.robot.crescendo.subsystems.shooter.util.ShooterIntakeDirection;

public class Cmd_SubSys_Shooter_ShootSpeakerFixedPosSpd extends Command {
  /** Creates a new Cmd_SubSys_Shooter_ShootSpeaker. */
  SubSys_Shooter subSysShooter;
  boolean atPos = false;
  boolean atSpd = false;
  boolean readyToShoot = false;
  boolean shooterEmpty = false;
  boolean doneShooting = false;
  Timer doneShootingTimer = new Timer();
 
  public Cmd_SubSys_Shooter_ShootSpeakerFixedPosSpd(SubSys_Shooter subSysShooter) {

    this.subSysShooter = subSysShooter;
    addRequirements(subSysShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subSysShooter.setShooterArmDegree(PresentArmPositions.ARM_PRESET_SPEAKER);
    atPos = false;

    subSysShooter.setShooterWheelsVelocity(ShooterWheels.SpeedSetPoints.SPEAKER_DEFAULT_SPD_CMD);
    atSpd = false;
    
    readyToShoot = false;
    shooterEmpty = false;
    doneShootingTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    atPos = subSysShooter.shooterArmMtrAtSetpoint();
    atSpd = subSysShooter.setShooterWheelsVelocity(ShooterWheels.SpeedSetPoints.SPEAKER_DEFAULT_SPD_CMD);
    
    if (!readyToShoot){
      if(atPos && atSpd){
        readyToShoot = true;
        subSysShooter.setIntakeOutput(ShooterIntakeDirection.SHOOT);
      }
    }else if(!shooterEmpty){
      if(!subSysShooter.getIntakeOccupied()){
        shooterEmpty = true;
        doneShootingTimer.start();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subSysShooter.stopAll();
    doneShootingTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return doneShootingTimer.hasElapsed(AutoAim.SHOOTER_WAIT_AFTER_SHOOT);
  }
}
