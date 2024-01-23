package frc.robot.crescendo.subsystems.intake;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SubSys_Intake extends SubsystemBase {
    private final TalonSRX spinner = new TalonSRX(9);
    private final DigitalInput beamSensor = new DigitalInput(0);
    private final DigitalInput forwardLimit = new DigitalInput(0);
    private final DigitalInput reverseLimit = new DigitalInput(0);
    private final TalonFX rotator = new TalonFX(0);
    private final TalonFXConfigurator talonFXConfigurator = rotator.getConfigurator();
    private final TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    private final HardwareLimitSwitchConfigs limitConfigs = new HardwareLimitSwitchConfigs();
    
    public SubSys_Intake () {
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.Feedback.FeedbackRemoteSensorID = 0;
        talonFXConfigs.Feedback.RotorToSensorRatio = 1;
        talonFXConfigurator.apply(talonFXConfigs);
    }

    public void lowerIntake() {
        rotator.setControl(new DutyCycleOut(SubSys_Intake_Constants.deploySpeed)
        .withLimitForwardMotion(forwardLimit.get())
        .withLimitReverseMotion(reverseLimit.get())
        .withOverrideBrakeDurNeutral(true));
    }

    public void liftIntake() {
        rotator.setControl(new DutyCycleOut(-SubSys_Intake_Constants.deploySpeed)
        .withLimitForwardMotion(forwardLimit.get())
        .withLimitReverseMotion(reverseLimit.get()));
    }

    public void setIntakeOutput(double percentOutput) {
        spinner.set(TalonSRXControlMode.PercentOutput, percentOutput);
    }

    public void setDeployerOutput(double percentOutput) {
        rotator.set(percentOutput);
    }

    public boolean getRingSensorValue() {
        return beamSensor.get();
    }
}