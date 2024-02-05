package frc.robot.crescendo.subsystems.shooter;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants;

/**
 * Handles outputs and inputs from the intake, including rotation motors and limit switches,
 * and Intake spinner.
 */
public class SubSys_Shooter extends SubsystemBase {

    private final TalonSRX shooterWheelsMtr1 = new TalonSRX(CAN_IDs.ShooterWheelsMtr1_CAN_ID);
    private final TalonSRX shooterWheelsMtr2 = new TalonSRX(CAN_IDs.ShooterWheelsMtr2_CAN_ID);
    private final CANSparkMax shooterRollerMtr = new CANSparkMax(CAN_IDs.ShooterRollerMtr_CAN_ID, MotorType.kBrushless);
    // private final DigitalInput beamSensor = new DigitalInput(0);
    private final TalonFX shooterArmMtr = new TalonFX(CAN_IDs.ShooterArmMtr_CAN_ID);
    private final TalonFXConfigurator shooterArmMtrConfigurator = shooterArmMtr.getConfigurator();
    private final TalonFXConfiguration shooterArmMtrConfiguration = new TalonFXConfiguration();
    private final CANcoder shooterArmCANCoder = new CANcoder(CAN_IDs.ShooterArmCANCoder_CAN_ID);
    private final CANcoderConfigurator shooterArmCANCoderConfigurator = shooterArmCANCoder.getConfigurator();
    private final CANcoderConfiguration shooterArmCaNcoderConfiguration = new CANcoderConfiguration();
    
    public SubSys_Shooter () {

        // Configure Shooter Arm Motor
        shooterArmMtrConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shooterArmMtrConfiguration.Feedback.FeedbackRemoteSensorID = shooterArmCANCoder.getDeviceID();
        shooterArmMtrConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        //shooterArmMtrConfiguration.Feedback.RotorToSensorRatio = 1;
        
        shooterArmMtrConfigurator.apply(shooterArmMtrConfiguration);

        // Configure Intake Arm CANcoder
        shooterArmCaNcoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        shooterArmCaNcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        shooterArmCaNcoderConfiguration.MagnetSensor.MagnetOffset = SubSys_Shooter_Constants.ShooterArm.CANcoderMagOffset;

        shooterArmCANCoderConfigurator.apply(shooterArmCaNcoderConfiguration);

    }

    /**
     * Set speed of shooter motor output. (-1)-1
     * @param percentOutput
     */
    public void setShooterOutput(double percentOutput) {
        shooterWheelsMtr1.set(TalonSRXControlMode.PercentOutput, percentOutput);
        shooterWheelsMtr2.set(TalonSRXControlMode.PercentOutput, percentOutput);   
    }

    public void setIntakeOutput(double percentOutput) {
        shooterRollerMtr.set(percentOutput);
    }

    /**
     * Set speed of shooterArmMtr motor output. (-1)-1
     * @param percentOutput
     */
    public void setShooterArmOutput(double percentOutput) {
        shooterArmMtr.set(percentOutput);
    }
}