package frc.robot.crescendo.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;

/**
 * Handles outputs and inputs from the intake, including rotation motors and limit switches,
 * and Intake spinner.
 */
public class SubSys_Shooter extends SubsystemBase {

    private final TalonSRX shooterWheelsMtr1 = new TalonSRX(CAN_IDs.ShooterWheelsMtrRight_CAN_ID);
    private final TalonSRX shooterWheelsMtr2 = new TalonSRX(CAN_IDs.ShooterWheelsMtrLeft_CAN_ID);
    private final CANSparkMax shooterRollerMtr = new CANSparkMax(CAN_IDs.ShooterRollerMtr_CAN_ID, MotorType.kBrushless);
    // private final DigitalInput beamSensor = new DigitalInput(0);
    private final TalonFX shooterArmMtr = new TalonFX(CAN_IDs.ShooterArmMtr_CAN_ID);
    private final CANcoder shooterArmCANCoder = new CANcoder(CAN_IDs.ShooterArmCANCoder_CAN_ID);

    // PIDs
    final PositionVoltage shooterArmPid;
    public SubSys_Shooter () {

        // Configure Shooter Arm Motor
        TalonFXConfiguration shooterArmMtrConfiguration = new TalonFXConfiguration();
        shooterArmMtrConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shooterArmMtrConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterArmMtrConfiguration.Feedback.FeedbackRemoteSensorID = shooterArmCANCoder.getDeviceID();
        shooterArmMtrConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        shooterArmMtrConfiguration.Slot0.kP = 0.5;
        shooterArmMtrConfiguration.Slot0.kI = 0;
        shooterArmMtrConfiguration.Slot0.kD = 0;
        //shooterArmMtrConfiguration.Feedback.RotorToSensorRatio = 1;

        // create a position closed-loop request, voltage output, slot 0 configs
        shooterArmPid = new PositionVoltage(0).withSlot(0);

        TalonFXConfigurator shooterArmMtrConfigurator = shooterArmMtr.getConfigurator();
        shooterArmMtrConfigurator.apply(shooterArmMtrConfiguration);

        // Configure Intake Arm CANcoder
        CANcoderConfiguration shooterArmCaNcoderConfiguration = new CANcoderConfiguration();
        shooterArmCaNcoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        shooterArmCaNcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        shooterArmCaNcoderConfiguration.MagnetSensor.MagnetOffset = SubSys_Shooter_Constants.ShooterArm.CANcoderMagOffset;

        CANcoderConfigurator shooterArmCANCoderConfigurator = shooterArmCANCoder.getConfigurator();
        shooterArmCANCoderConfigurator.apply(shooterArmCaNcoderConfiguration);

    }

    // Intake ------
    /**
     * Set speed of shooter motor output. (-1)-1
     * @param percentOutput -1 -> 1
     */
    public void setShooterOutput(double percentOutput) {
        shooterWheelsMtr1.set(TalonSRXControlMode.PercentOutput, percentOutput);
        shooterWheelsMtr2.set(TalonSRXControlMode.PercentOutput, percentOutput);   
    }

    public void setIntakeOutput(double percentOutput) {
        shooterRollerMtr.set(percentOutput);
    }


    // Arm ------
    /**
     * Set speed of shooterArmMtr motor output. (-1)-1
     * @param percentOutput -1 -> 1
     */
    public void setShooterArmOutput(double percentOutput) {
        shooterArmMtr.set(percentOutput);
    }

    /**
     * Set the degree of the arm rotation
     *
     * @param degree The degree to rotate to
     */
    public void setShooterArmDegree(double degree) {
        shooterArmMtr.setControl(shooterArmPid.withPosition(degree / 360.0));
    }

    /**
     * @return If the motors velocity does not equal 0
     */
    public boolean shooterArmMtrBusy() {
        return (shooterArmMtr.getVelocity().getValueAsDouble() != 0);
    }

}