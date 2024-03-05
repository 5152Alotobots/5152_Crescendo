// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;

public class SubSys_Climber extends SubsystemBase {

    private final TalonFX climberLeftMtr = new TalonFX(CAN_IDs.ClimberLeftMtr_CAN_ID, "canivore");
    private final TalonFX climberRightMtr = new TalonFX(CAN_IDs.ClimberRightMtr_CAN_ID, "canivore");

    // PIDs
    final PositionVoltage climberLeftPid;
    final PositionVoltage climberRightPid;

    // Climber Support
    private final DoubleSolenoid climberSupport_DoubleSolenoid =
        new DoubleSolenoid(2,PneumaticsModuleType.CTREPCM, 2, 3);

    /**
     * Creates a new SubSys_Climber.
     */

    public SubSys_Climber() {

        // ---- Configure climber Left Motor
          
        // Setup Configuration
        TalonFXConfiguration climberLeftMtrConfiguration = new TalonFXConfiguration();
        climberLeftMtrConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberLeftMtrConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climberLeftMtrConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        climberLeftMtrConfiguration.Feedback.SensorToMechanismRatio = SubSys_Climber_Constants.climberSensorToMechanismRatio;
        climberLeftMtrConfiguration.Slot0.kP = 0.5;
        climberLeftMtrConfiguration.Slot0.kI = 0;
        climberLeftMtrConfiguration.Slot0.kD = 0;
        climberLeftMtrConfiguration.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        climberLeftMtrConfiguration.HardwareLimitSwitch.ForwardLimitEnable = true;
        climberLeftMtrConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
        climberLeftMtrConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = SubSys_Climber_Constants.climberLeftMaxHeight;
        climberLeftMtrConfiguration.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        climberLeftMtrConfiguration.HardwareLimitSwitch.ReverseLimitEnable = true;
        climberLeftMtrConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;
        climberLeftMtrConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = SubSys_Climber_Constants.climberLeftMinHeight;

        // create a position closed-loop request, voltage output, slot 0 configs
        climberLeftPid = new PositionVoltage(0).withSlot(0);
          
        TalonFXConfigurator climberLeftMtrConfigurator = climberLeftMtr.getConfigurator();
        climberLeftMtrConfigurator.apply(climberLeftMtrConfiguration);


        // Configure climber Left Motor
        TalonFXConfiguration climberRightMtrConfiguration = new TalonFXConfiguration();
        climberRightMtrConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberRightMtrConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        climberRightMtrConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        climberRightMtrConfiguration.Feedback.SensorToMechanismRatio = SubSys_Climber_Constants.climberSensorToMechanismRatio;
        climberRightMtrConfiguration.Slot0.kP = 0.5;
        climberRightMtrConfiguration.Slot0.kI = 0;
        climberRightMtrConfiguration.Slot0.kD = 0;
        climberRightMtrConfiguration.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        climberRightMtrConfiguration.HardwareLimitSwitch.ForwardLimitEnable = true;
        climberRightMtrConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        climberRightMtrConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = SubSys_Climber_Constants.climberRightMaxHeight;
        climberRightMtrConfiguration.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        climberRightMtrConfiguration.HardwareLimitSwitch.ReverseLimitEnable = true;
        climberRightMtrConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;
        climberRightMtrConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = SubSys_Climber_Constants.climberRightMinHeight;

        // create a position closed-loop request, voltage output, slot 0 configs
        climberRightPid = new PositionVoltage(0).withSlot(0);

        TalonFXConfigurator climberRightMtrConfigurator = climberRightMtr.getConfigurator();
              climberRightMtrConfigurator.apply(climberRightMtrConfiguration);  

        
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      SmartDashboard.putNumber("climberLeftMtrPos", climberLeftMtr.getPosition().getValueAsDouble());
      SmartDashboard.putBoolean("climberLeftMtrFwdLimit", climberLeftMtr.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround);
      SmartDashboard.putBoolean("climberLeftMtrRevLimit", climberLeftMtr.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround);
      SmartDashboard.putNumber("climberRightMtrPos", climberRightMtr.getPosition().getValueAsDouble());
      SmartDashboard.putBoolean("climberRightMtrFwdLimit", climberRightMtr.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround);
      SmartDashboard.putBoolean("climberRightMtrRevLimit", climberRightMtr.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround);

  }

   @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run
  }

  public void setVoltCmd(double voltCmd){
    final VoltageOut requestVoltOut = new VoltageOut(0);
    climberLeftMtr.setControl(requestVoltOut.withOutput(voltCmd));
    climberRightMtr.setControl(requestVoltOut.withOutput(voltCmd));
  }

    public boolean setPositionCmd(double posCmd) {
        final PositionVoltage requestPosVolt = new PositionVoltage(0);
        //climberLeftMtr.setControl(requestPosVolt.withPosition(posCmd));
        //climberRightMtr.setControl(requestPosVolt.withPosition(posCmd));
        return false;
  }

  public void climberSupportExtendCmd() {
    climberSupport_DoubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void climberSupportRetractCmd() {
    climberSupport_DoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

}