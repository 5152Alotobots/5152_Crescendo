// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.subsystems.slider;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubSys_Slider extends SubsystemBase {
  /** Creates a new SubSys_Slider. */

  private final DoubleSolenoid slider_leftDoubleSolenoid =
      new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
        
  private final DoubleSolenoid slider_rightDoubleSolenoid =
      new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
       
  private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);

  public SubSys_Slider() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void sliderExtendCmd() {
    slider_leftDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
    slider_rightDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void sliderRetractCmd() {
    slider_leftDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    slider_rightDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
}
