// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;

public class SubSys_Climber extends SubsystemBase {

 private final TalonFX climberLeftMtr = new TalonFX(CAN_IDs.ClimberLeftMtr_CAN_ID);
 private final TalonFX climberRightMtr = new TalonFX(CAN_IDs.ClimberRightMtr_CAN_ID);

// PIDs
final PositionVoltage climberLeftPid;
final PositionVoltage climberRightPid;

    /** Creates a new SubSys_Slider. */
    public SubSys_Climber() {


        // Configure climber Left Motor
        TalonFXConfiguration climberLeftMtrConfiguration = new TalonFXConfiguration();
        climberLeftMtrConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberLeftMtrConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climberLeftMtrConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        climberLeftMtrConfiguration.Slot0.kP = 0.5;
        climberLeftMtrConfiguration.Slot0.kI = 0;
        climberLeftMtrConfiguration.Slot0.kD = 0;
        //climberLeftMtrConfiguration.Feedback.RotorToSensorRatio = 1;

        // create a position closed-loop request, voltage output, slot 0 configs
        climberLeftPid = new PositionVoltage(0).withSlot(0);
        
        TalonFXConfigurator climberLeftMtrConfigurator = climberLeftMtr.getConfigurator();
        climberLeftMtrConfigurator.apply(climberLeftMtrConfiguration);


         // Configure climber Left Motor
        TalonFXConfiguration climberRightMtrConfiguration = new TalonFXConfiguration();
        climberRightMtrConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberRightMtrConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climberRightMtrConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        climberRightMtrConfiguration.Slot0.kP = 0.5;
        climberRightMtrConfiguration.Slot0.kI = 0;
        climberRightMtrConfiguration.Slot0.kD = 0;
        //climberLeftMtrConfiguration.Feedback.RotorToSensorRatio = 1;

        // create a position closed-loop request, voltage output, slot 0 configs
        climberRightPid = new PositionVoltage(0).withSlot(0);

        TalonFXConfigurator climberRightMtrConfigurator = climberRightMtr.getConfigurator();
        climberRightMtrConfigurator.apply(climberRightMtrConfiguration);  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}