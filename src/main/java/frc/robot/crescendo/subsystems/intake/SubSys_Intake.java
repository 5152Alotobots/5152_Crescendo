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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.DigitalIO_IDs;
import frc.robot.library.driverstation.JoystickUtilities;

/**
 * Handles outputs and inputs from the intake, including rotation motors and limit switches,
 * and Intake intakeRollerMtr.
 */
public class SubSys_Intake extends SubsystemBase {
    private final CANSparkMax intakeRollerMtr = new CANSparkMax(CAN_IDs.IntakeRollerMtr_CAN_ID, MotorType.kBrushless);
    private final DigitalInput intakeRollerIR = new DigitalInput(DigitalIO_IDs.IntakeRollerIRDetector_ID);
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
        CANcoderConfiguration intakeArmCANcoderConfiguration = new CANcoderConfiguration();
        intakeArmCANcoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        intakeArmCANcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        intakeArmCANcoderConfiguration.MagnetSensor.MagnetOffset = SubSys_Intake_Constants.IntakeArm.CANcoderMagOffset;

        CANcoderConfigurator intakeArmCANCoderConfigurator = intakeArmCANCoder.getConfigurator();
        intakeArmCANCoderConfigurator.apply(intakeArmCANcoderConfiguration);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putString("Intake/Arm Forward Value", intakeArmMtr.getForwardLimit().toString());
        SmartDashboard.putString("Intake/Arm Reverse Value", intakeArmMtr.getReverseLimit().toString());
        SmartDashboard.putBoolean("Intake/IR Raw value", intakeRollerIR.get());
        SmartDashboard.putBoolean("Intake/Intake Occupied", getIntakeOccupied());
        SmartDashboard.putNumber("Intake/Arm Speed", intakeArmMtr.get());
    }

    /**
     * Calls correct IntakeArm method depending on the values of limit switches
     * @param speed Speed from -1 - 1 (unscaled)
     */
    public void setIntakeArmSpeedWithLimits(double intakeArmSpeed) {
        ForwardLimitValue forwardLimitValue = intakeArmMtr.getForwardLimit().getValue();
        ReverseLimitValue reverseLimitValue = intakeArmMtr.getReverseLimit().getValue();
        
        if (forwardLimitValue == ForwardLimitValue.ClosedToGround) {
            SmartDashboard.putBoolean("Intake/ForwardLimitMode", true);
            SmartDashboard.putBoolean("Intake/ReverseLimitMode", false);
            SmartDashboard.putBoolean("Intake/NoLimitMode", false);
            liftIntakeArmSpeed(intakeArmSpeed);
        } else if (reverseLimitValue == ReverseLimitValue.ClosedToGround) {
            SmartDashboard.putBoolean("Intake/ForwardLimitMode", false);
            SmartDashboard.putBoolean("Intake/ReverseLimitMode", true);
            SmartDashboard.putBoolean("Intake/NoLimitMode", false);
            lowerIntakeArmSpeed(intakeArmSpeed);
        } else if (forwardLimitValue == ForwardLimitValue.Open && reverseLimitValue == ReverseLimitValue.Open) {
            SmartDashboard.putBoolean("Intake/ForwardLimitMode", false);
            SmartDashboard.putBoolean("Intake/ReverseLimitMode", false);
            SmartDashboard.putBoolean("Intake/NoLimitMode", true);
            setIntakeArmSpeed(intakeArmSpeed);
        }
    }

    /**
     * Calls setIntakeArmSpeed with a Math.min to make sure the value is not negitive
     * @param speed Speed from -1 - 1 (unscaled, anything positive will be ignored)
     */
    public void lowerIntakeArmSpeed(double speed) { 
        setIntakeArmSpeed(Math.max(0, speed));
    }

    /**
     * Calls setIntakeArmSpeed with a Math.max to make sure the value is not negitive
     * @param speed Speed from -1 - 1 (unscaled, anything negative will be ignored)
     */
    public void liftIntakeArmSpeed(double speed) {
        setIntakeArmSpeed(Math.min(0, speed));
    }

    /**
     * Takes a speed scales (Applies deadband as well) it and sends it to the motor
     * @param speed Speed from -1 - 1 (unscaled)
     */
    public void setIntakeArmSpeed(double speed) {
        SmartDashboard.putNumber("Intake/Input Scaled", JoystickUtilities.joyDeadBndScaled(speed, 0.25, 0.2));
        intakeArmMtr.set(JoystickUtilities.joyDeadBndSqrdScaled(speed, 0.25, 0.2));
    }

    /**
     * @param intakeSpeed {@link IntakeSpeed} - The speed to run the intake at
     */
    public void setIntakeDirection(IntakeDirection intakeDirection) {
        SmartDashboard.putString("Direction Intake", intakeDirection.toString());
        if (intakeDirection == IntakeDirection.IN && getIntakeOccupied()) {
               SmartDashboard.putBool tan("Intake/intake", true);
            intakeRollerMtr.set(1);
        } else if (intakeDirection == IntakeDirection.OUT) {
                  SmartDashboard.putBoolean("Intake/outake", true);
            intakeRollerMtr.set(-1);
        } else intakeRollerMtr.set(0);
    }

    /**
     * @return true if the intake is occupied with a note
     * */
    public boolean getIntakeOccupied() {
        return !intakeRollerIR.get();
    }
}