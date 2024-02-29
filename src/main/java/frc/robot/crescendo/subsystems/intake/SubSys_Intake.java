package frc.robot.crescendo.subsystems.intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.DigitalIO_IDs;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.IntakeArm;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.IntakeRoller;
import frc.robot.library.driverstation.JoystickUtilities;

import static frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.IntakeArm.Arm.*;
import static frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.Limit.*;
import static frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.MaxSpeeds.MAX_INTAKE_SPEED;
import static frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.MaxSpeeds.TRANSFER_SPEED;
import static frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.PresetIntakePositions.INTAKE_ARM_POSITION_TOLERANCE;

/**
 * Handles outputs and inputs from the intake, including rotation motors and limit switches,
 * and Intake intakeRollerMtr.
 */
public class SubSys_Intake extends SubsystemBase {
    private final CANSparkMax intakeRollerMtr = new CANSparkMax(CAN_IDs.IntakeRollerMtr_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final SparkPIDController intakeRollerMtrPID;
    private final RelativeEncoder intakeRollerMtrEncoder;

    private final DigitalInput intakeRollerIR = new DigitalInput(DigitalIO_IDs.IntakeRollerIRDetector_ID);
    private final TalonFX intakeArmMtr = new TalonFX(CAN_IDs.IntakeArmMtr_CAN_ID);
    private final CANcoder intakeArmCANCoder = new CANcoder(CAN_IDs.IntakeArmCANCoder_CAN_ID);
    private final Timer timer = new Timer();
    private double intakeRollerMtrSetpoint = 0.0;

    final PositionVoltage intakeArmPid;
    public SubSys_Intake () {
        
        intakeRollerMtr.restoreFactoryDefaults();
        intakeRollerMtr.enableVoltageCompensation(10);
        intakeRollerMtr.setInverted(false);
        intakeRollerMtr.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intakeRollerMtr.setOpenLoopRampRate(IntakeRoller.OpenLoopRampRate);
        intakeRollerMtr.setClosedLoopRampRate(IntakeRoller.PID.ClosedLoopRampRate);

        intakeRollerMtrEncoder = intakeRollerMtr.getEncoder();

        intakeRollerMtrPID = intakeRollerMtr.getPIDController();
        intakeRollerMtrPID.setP(IntakeRoller.PID.Pgain);
        intakeRollerMtrPID.setI(IntakeRoller.PID.Igain);
        intakeRollerMtrPID.setD(IntakeRoller.PID.Dgain);
        intakeRollerMtrPID.setIZone(IntakeRoller.PID.Izone);
        intakeRollerMtrPID.setFF(IntakeRoller.PID.FFwd);
        intakeRollerMtrPID.setOutputRange(IntakeRoller.PID.MinOutput,IntakeRoller.PID.MaxOutput);

        // Configure Intake Arm Motor
        TalonFXConfiguration intakeArmMtrConfiguration = new TalonFXConfiguration();
        intakeArmMtrConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeArmMtrConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeArmMtrConfiguration.Feedback.FeedbackRemoteSensorID = intakeArmCANCoder.getDeviceID();
        intakeArmMtrConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        intakeArmMtrConfiguration.Slot0.kP = 0.5;
        intakeArmMtrConfiguration.Slot0.kI = 0;
        intakeArmMtrConfiguration.Slot0.kD = 0;
        intakeArmMtrConfiguration.Feedback.RotorToSensorRatio = 1; // 0.005291;
        intakeArmMtrConfiguration.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        intakeArmMtrConfiguration.HardwareLimitSwitch.ForwardLimitEnable = true;
        intakeArmMtrConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
        intakeArmMtrConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = IntakeArm.FwdLimitSwitchPos;
        intakeArmMtrConfiguration.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        intakeArmMtrConfiguration.HardwareLimitSwitch.ReverseLimitEnable = true;
        intakeArmMtrConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;
        intakeArmMtrConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = IntakeArm.RevLimitSwitchPos;
        intakeArmMtrConfiguration.Slot0.kP = ARM_P;
        intakeArmMtrConfiguration.Slot0.kI = ARM_I;
        intakeArmMtrConfiguration.Slot0.kD = ARM_D;
        intakeArmMtrConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ARM_LIMIT_FORWARD;
        intakeArmMtrConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ARM_LIMIT_REVERSE;
        intakeArmMtrConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = ARM_LIMIT_ENABLE;
        intakeArmMtrConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = ARM_LIMIT_ENABLE;
        // create a position closed-loop request, voltage output, slot 0 configs
        intakeArmPid = new PositionVoltage(0).withSlot(0);
        TalonFXConfigurator intakeArmMtrConfigurator = intakeArmMtr.getConfigurator();
        intakeArmMtrConfigurator.apply(intakeArmMtrConfiguration);
        
        // Configure Intake Arm CANcoder
        CANcoderConfiguration intakeArmCANcoderConfiguration = new CANcoderConfiguration();
        intakeArmCANcoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        intakeArmCANcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        intakeArmCANcoderConfiguration.MagnetSensor.MagnetOffset = SubSys_Intake_Constants.IntakeArm.CANcoderMagOffset;

        CANcoderConfigurator intakeArmCANCoderConfigurator = intakeArmCANCoder.getConfigurator();

        // CANcoderConfigurator intakeArmCANCoderConfigurator = intakeArmCANCoder.getConfigurator();
        // intakeArmCANCoderConfigurator.apply(intakeArmCANcoderConfiguration);

        intakeRollerMtr.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }
    
    @Override
    public void periodic() {
        // Intake Roller Motor
        //SmartDashboard.putNumber("IntakeRollerVelSetPoint", intakeRollerMtrSetpoint);
        //SmartDashboard.putNumber("IntakeRollerVel", intakeRollerMtrEncoder.getVelocity());

        //SmartDashboard.putString("Intake/Arm Forward Value", intakeArmMtr.getForwardLimit().toString());
        //SmartDashboard.putString("Intake/Arm Reverse Value", intakeArmMtr.getReverseLimit().toString());
        //SmartDashboard.putBoolean("Intake/IR Raw value", intakeRollerIR.get());
        SmartDashboard.putBoolean("Intake/Intake Occupied", getIntakeOccupied());
        // SmartDashboard.putNumber("Intake/Can Coder Abs", intakeArmCANCoder.getAbsolutePosition().getValueAsDouble());
        //SmartDashboard.putNumber("Intake/Arm Motor Remote", intakeArmMtr.getPosition().getValueAsDouble());
        //SmartDashboard.putNumber("Intake/Arm Speed", intakeArmMtr.get());
        //SmartDashboard.putNumber("Intake/Arm Can Coder Abs", intakeArmCANCoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("IntakeArmEncoderAbsolutePos", intakeArmCANCoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("IntakeArmEncoderPos", intakeArmCANCoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("IntakeArmMtrPos", intakeArmMtr.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("IntakeArmPos", getIntakeArmPos());
        SmartDashboard.putBoolean("Intake/Intake Arm Motor Busy", intakeArmMtrBusy());
        SmartDashboard.putBoolean("Intake/Intake Arm Motor At Setpoint", intakeArmMtrAtSetpoint());
        SmartDashboard.putNumber("Intake/Intake Arm PID Position", intakeArmMtr.getClosedLoopReference().getValueAsDouble());

    }

    /**
     * Calls correct IntakeArm method depending on the values of limit switches
     */
    public void setIntakeArmSpeedWithLimits(double intakeArmSpeed) {
        ForwardLimitValue forwardLimitValue = intakeArmMtr.getForwardLimit().getValue();
        ReverseLimitValue reverseLimitValue = intakeArmMtr.getReverseLimit().getValue();
        
        if (forwardLimitValue == ForwardLimitValue.ClosedToGround) {
            //SmartDashboard.putBoolean("Intake/ForwardLimitMode", true);
            //SmartDashboard.putBoolean("Intake/ReverseLimitMode", false);
            //SmartDashboard.putBoolean("Intake/NoLimitMode", false);
            liftIntakeArmSpeed(intakeArmSpeed);
        } else if (reverseLimitValue == ReverseLimitValue.ClosedToGround) {
            //SmartDashboard.putBoolean("Intake/ForwardLimitMode", false);
            //SmartDashboard.putBoolean("Intake/ReverseLimitMode", true);
            //SmartDashboard.putBoolean("Intake/NoLimitMode", false);
            lowerIntakeArmSpeed(intakeArmSpeed);
        } else if (forwardLimitValue == ForwardLimitValue.Open && reverseLimitValue == ReverseLimitValue.Open) {
            //SmartDashboard.putBoolean("Intake/ForwardLimitMode", false);
            //SmartDashboard.putBoolean("Intake/ReverseLimitMode", false);
            //SmartDashboard.putBoolean("Intake/NoLimitMode", true);
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
        SmartDashboard.putNumber("Intake/Input Scaled", JoystickUtilities.joyDeadBndScaled(speed, 0.25, 0.4));
        intakeArmMtr.set(JoystickUtilities.joyDeadBndSqrdScaled(speed, 0.25, 0.6));
    }

    /**
     * @param intakeDirection {@link IntakeDirection} - The speed to run the intake at
     */
    public void setIntakeDirection(IntakeDirection intakeDirection) {
        SmartDashboard.putString("Intake/Direction Intake", intakeDirection.toString());
        double speed = 0;
        switch (intakeDirection) {
            case IN:
                if (!getIntakeOccupied()) speed = MAX_INTAKE_SPEED;
                else speed = 0;
                break;
            case TRANSFER:
                speed = TRANSFER_SPEED;
                break;
            case OUT:
                speed = -MAX_INTAKE_SPEED;
                break;
            default:
                break;
        }
        intakeRollerMtr.set(speed);
    }

    /**
     * Set the degree of the arm rotation
     * @param degree The degree to rotate to
     */
    // public void setShooterArmDegree(double degree) {
    //     SmartDashboard.putNumber("Intake/Intake Arm Target Position", degree);
    //     intakeArmMtr.setControl(new PositionVoltage(degree / 360.0).withSlot(0));
    // }

    /**
     * @return true if the intake is occupied with a note
     * */
    public boolean getIntakeOccupied() {
        return !intakeRollerIR.get();
    }


    public void setIntakeRollerSpdDutyCycle(double spdCmd){
        intakeRollerMtr.set(spdCmd);
    }

    public void setIntakeRollerSpdRPM(double spdCmd){
        intakeRollerMtrPID.setReference(spdCmd, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * @deprecated Use setIntakeDirection instead
     */
    @Deprecated
    public void intakeNote(){
        if (getIntakeOccupied()){
            intakeRollerMtr.set(0);
        } else {
            intakeRollerMtr.set(IntakeRoller.intakeNoteSpeed);
        }
    }

    /**
     * @deprecated Use setIntakeDirection instead
     */
    @Deprecated
    public void ejectNote(){
        intakeRollerMtr.set(IntakeRoller.ejectNoteSpeed);
    }

    /**
     * @deprecated Use setIntakeDirection instead
     */
    @Deprecated
    public void transferNote(){
        intakeRollerMtr.set(IntakeRoller.transferNoteSpeed);
    }

    public void setIntakeArmSpd(double spdCmd){
        intakeArmMtr.set(spdCmd);
    }


    public double getIntakeArmPos(){
        return intakeArmMtr.getPosition().getValueAsDouble();
    }

    /**
     * @deprecated Use {@link SubSys_Intake } setIntakeArmDegree instead
     */
    @Deprecated
    public boolean setIntakeArmPosCmd(double posCmd){
        boolean atPos = false;
        double error = posCmd-getIntakeArmPos();
        if(error > 0.015){
            intakeArmMtr.set(IntakeArm.IntakeArmPosCmdSpd);
            atPos = false;
        } else if (error < 0.015) {
            intakeArmMtr.set(-1 * IntakeArm.IntakeArmPosCmdSpd);
            atPos = false;
        } else {
            intakeArmMtr.set(0.0);
            atPos = true;
        }
        return atPos;
    }

    /**
     * Set the degree of the intake arm rotation
     *
     * @param degree The degree to rotate to (negative degrees angles away from intake -90 = straight up)
     */
    public void setIntakeArmDegree(double degree) {
        SmartDashboard.putNumber("Intake/Intake Arm Target Position", degree);
        double limitAdjusted = MathUtil.clamp(degree / 360.0, ARM_LIMIT_REVERSE * 360, ARM_LIMIT_FORWARD * 360); // Limit to motor limits
        intakeArmMtr.setControl(intakeArmPid.withPosition(limitAdjusted));
    }

    /**
     * @return true if the motor is within position tolerance
     */
    public boolean intakeArmMtrAtSetpoint() {
        return (Math.abs((intakeArmMtr.getPosition().getValueAsDouble() - intakeArmMtr.getClosedLoopReference().getValueAsDouble())) <= INTAKE_ARM_POSITION_TOLERANCE);
    }

    /**
     * @return true if the motors velocity does not equal zero
     */
    public boolean intakeArmMtrBusy() {
        return (intakeArmMtr.getVelocity().getValueAsDouble() != 0);
    }

    /**
     * @return true if the intake is at the upper limit
     */
    public boolean atUpperLimit() {
        return intakeArmMtr.getReverseLimit().getValue().equals(ReverseLimitValue.ClosedToGround);
    }

    /**
     * @return true if the intake is at the lower limit
     */
    public boolean atLowerLimit() {
        return intakeArmMtr.getForwardLimit().getValue().equals(ForwardLimitValue.ClosedToGround);
    }
}