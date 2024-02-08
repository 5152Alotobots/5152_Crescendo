package frc.robot.crescendo.subsystems.intake;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
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
    private final CANcoder intakeArmCANCoder = new CANcoder(CAN_IDs.IntakeArmCANCoder_CAN_ID);

    public SubSys_Intake () {
        
        // Configure Intake Arm Motor
        TalonFXConfiguration intakeArmMtrConfiguration = new TalonFXConfiguration();
        intakeArmMtrConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeArmMtrConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeArmMtrConfiguration.Feedback.FeedbackRemoteSensorID = intakeArmCANCoder.getDeviceID();
        intakeArmMtrConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        intakeArmMtrConfiguration.Slot0.kP = 0.5;
        intakeArmMtrConfiguration.Slot0.kI = 0;
        intakeArmMtrConfiguration.Slot0.kD = 0;
        //intakeArmMtrConfiguration.Feedback.RotorToSensorRatio = 1;
        TalonFXConfigurator intakeArmMtrConfigurator = intakeArmMtr.getConfigurator();
        intakeArmMtrConfigurator.apply(intakeArmMtrConfiguration);
        
        // Configure Intake Arm CANcoder
        CANcoderConfiguration intakeArmCaNcoderConfiguration = new CANcoderConfiguration();
        intakeArmCaNcoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        intakeArmCaNcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        intakeArmCaNcoderConfiguration.MagnetSensor.MagnetOffset = SubSys_Intake_Constants.IntakeArm.CANcoderMagOffset;

        CANcoderConfigurator intakeArmCANCoderConfigurator = intakeArmCANCoder.getConfigurator();
        intakeArmCANCoderConfigurator.apply(intakeArmCaNcoderConfiguration);
    }
    @Override
    public void periodic() {
    }

    public void setIntakeArmPosition(IntakePosition intakePosition) {
        if (intakePosition == IntakePosition.UP && !atUpperLimit()) {
            intakeRollerMtr.set(1);
        } else if (intakePosition == IntakePosition.DOWN && !atLowerLimit()) {
            intakeRollerMtr.set(-1);
        } else {
            intakeRollerMtr.set(0);
        }
    }

    /**
     * @param intakeSpeed {@link IntakeSpeed} - The speed to run the intake at
     */
    public void setIntakeSpeed(IntakeSpeed intakeSpeed) {
        if (intakeSpeed == IntakeSpeed.IN && !getIntakeOccupied()) {
            intakeRollerMtr.set(1);
        } else if (intakeSpeed == IntakeSpeed.OUT) {
            intakeRollerMtr.set(-1);
        } else intakeRollerMtr.set(0);
    }

    /**
     * @return true if the intake is occupied with a note
     * */
    public boolean getIntakeOccupied() {
        return !intakeRollerIRDetector.get();
    }


    public boolean atUpperLimit() {
        return intakeArmFwdLimitSw.get();
    }

    public boolean atLowerLimit() {
        return intakeArmRevLimitSw.get();
    }
}