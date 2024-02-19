// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.subsystems.slider;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubSys_Slider extends SubsystemBase {
  /** Creates a new SubSys_Slider. */
  public SubSys_Slider() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public final DoubleSolenoid extend =
    new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM, 0, 1);

    public void retract() {
    extend.set(Value.kReverse);
  }

  public void extend() {
    extend.set(Value.kForward);
  }

  public Value getHandState() {
    return extend.get();
  }
}
