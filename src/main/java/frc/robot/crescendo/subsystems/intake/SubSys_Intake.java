package frc.robot.crescendo.subsystems.intake;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.Orchestra;
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
import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.DigitalIO_IDs;

/**
 * Handles outputs and inputs from the intake, including rotation motors and limit switches,
 * and Intake intakeRollerMtr.
 */
public class SubSys_Intake extends SubsystemBase {

    private final Orchestra orchestra = new Orchestra();
    private final CANSparkMax intakeRollerMtr = new CANSparkMax(CAN_IDs.IntakeRollerMtr_CAN_ID, MotorType.kBrushless);
    private final DigitalInput intakeRollerIRDetector = new DigitalInput(DigitalIO_IDs.IntakeRollerIRDetector_ID);
    private final DigitalInput intakeArmFwdLimitSw = new DigitalInput(DigitalIO_IDs.IntakeArmFwdLimitSw_ID);
    private final DigitalInput intakeArmRevLimitSw = new DigitalInput(DigitalIO_IDs.IntakeArmRevLimitSw_ID);
    private final TalonFX intakeArmMtr = new TalonFX(CAN_IDs.IntakeArmMtr_CAN_ID);
    private final TalonFXConfigurator intakeArmMtrConfigurator = intakeArmMtr.getConfigurator();
    private final TalonFXConfiguration intakeArmMtrConfiguration = new TalonFXConfiguration();
    private final CANcoder intakeArmCANCoder = new CANcoder(CAN_IDs.IntakeArmCANCoder_CAN_ID);
    private final CANcoderConfigurator intakeArmCANCoderConfigurator = intakeArmCANCoder.getConfigurator();
    private final CANcoderConfiguration intakeArmCaNcoderConfiguration = new CANcoderConfiguration();

    public SubSys_Intake () {
        
        // Configure Intake Arm Motor
        intakeArmMtrConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeArmMtrConfiguration.Feedback.FeedbackRemoteSensorID = intakeArmCANCoder.getDeviceID();
        intakeArmMtrConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        //intakeArmMtrConfiguration.Feedback.RotorToSensorRatio = 1;

        intakeArmMtrConfigurator.apply(intakeArmMtrConfiguration);
        
        // Configure Intake Arm CANcoder
        intakeArmCaNcoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        intakeArmCaNcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        intakeArmCaNcoderConfiguration.MagnetSensor.MagnetOffset = SubSys_Intake_Constants.IntakeArm.CANcoderMagOffset;

        intakeArmCANCoderConfigurator.apply(intakeArmCaNcoderConfiguration);


        // orchestra.addInstrument(intakeArmMtr);
        // orchestra.loadMusic("finalCountdown.chrp");
        // orchestra.play();
        
    }

    public void lowerDeployer(double speed) { 
        intakeArmMtr.set(Math.min(0, speed));
    }

    public void liftDeployer(double speed) {
        intakeArmMtr.set(Math.max(0, speed));
    }

    public void setDeployer(double speed) {
        intakeArmMtr.set(speed);
    }

    /**
     * Set speed of intake motor output. (-1)-1
     * @param percentOutput
     */
    public void setIntakeOutput(double percentOutput) {
        intakeRollerMtr.set(percentOutput);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("El sensor de note", getRingSensorValue());
        SmartDashboard.putBoolean("El sensor de forward", getForwardSensorValue());
        SmartDashboard.putBoolean("El sensor de reverse", getReverseSensorValue());
        SmartDashboard.putNumber("intakeArmMtr Output", intakeArmMtr.get());
    }

    /**
     * Set speed of deployer motor output. (-1)-1
     * @param percentOutput
     */
    // public void setDeployerOutput(double percentOutput) {
    //     intakeArmMtr.set(percentOutput);
    // }

    /**
     * Get value of ring sensor
     * @return 
     */
    public boolean getRingSensorValue() {
        return intakeRollerIRDetector.get();
    }

    public boolean getForwardSensorValue() {
        return intakeArmFwdLimitSw.get();
    }

     public boolean getReverseSensorValue() {
        return intakeArmRevLimitSw.get();
    }
}