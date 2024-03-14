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

public class Cmd_SubSys_Shooter_AimSpinShoot extends Command {
  /** Creates a new Cmd_SubSys_Shooter_ShootSpeaker. */
  SubSys_Shooter subSysShooter;
  double shooterArmPosCmd;
  double shooterWheelsSpdCmd;
  boolean atPos = false;
  boolean atSpd = false;
  boolean readyToShoot = false;
  boolean noteInShooter = true;

  Timer timer = new Timer();

  public Cmd_SubSys_Shooter_AimSpinShoot(
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

    readyToShoot = false;
    noteInShooter = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    atPos = subSysShooter.shooterArmMtrAtSetpoint();
    subSysShooter.setShooterWheelsVelocityVolts(shooterWheelsSpdCmd);
    
    double leftWheelsVelError = shooterWheelsSpdCmd - subSysShooter.getShooterLeftWheelsVelocity();
    double rightWheelsVelError = shooterWheelsSpdCmd - subSysShooter.getShooterRightWheelsVelocity();
    boolean leftWheelsVelAtSpd = leftWheelsVelError < 5;
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
      subSysShooter.setIntakeOutput(ShooterIntakeDirection.SHOOT);
    }
    
    if(noteInShooter && !subSysShooter.getIntakeOccupied()){
      timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subSysShooter.stopAll();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !subSysShooter.getIntakeOccupied() && timer.hasElapsed(AutoAim.SHOOTER_WAIT_AFTER_SHOOT);
  }
}
