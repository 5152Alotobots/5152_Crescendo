package frc.robot.crescendo.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import static edu.wpi.first.units.Units.Volts;
/**
 * Handles outputs and inputs from the intake, including rotation motors and limit switches,
 * and Intake intakeRollerMtr.
 */

public class SubSys_Intake extends SubsystemBase {
    // ********************
    // Intake Roller
    // ********************
    // Intake Roller - Motor
    private final CANSparkMax intakeRollerMtr = new CANSparkMax(CAN_IDs.IntakeRollerMtr_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);
    // Intake Roller - Encoder
    private final RelativeEncoder intakeRollerMtrEncoder;
    // Intake Roller - Motor PID
    private final SparkPIDController intakeRollerMtrPID;
    // Intake Roller IR Sensor
    private final DigitalInput intakeRollerIR = new DigitalInput(DigitalIO_IDs.IntakeRollerIRDetector_ID);
    
    // ********************
    // Intake Arm
    // ********************
    // Intake Arm Motor
    private final TalonFX intakeArmMtr = new TalonFX(CAN_IDs.IntakeArmMtr_CAN_ID);
    // Intake Arm CANcoder
    private final CANcoder intakeArmCANCoder = new CANcoder(CAN_IDs.IntakeArmCANCoder_CAN_ID);
    // Intake Arm Commands
    private final VoltageOut intakeArmMtrVoltOutCmd = new VoltageOut(0.0); 
    private final VelocityVoltage intakeArmMtrVelVoltCmd;
    private final MotionMagicVoltage intakeArmMtrVoltMMCmd;
    private final MotionMagicVelocityVoltage intakeArmMtrVelVoltMMCmd;
    private final ArmFeedforward intakeArmFFVoltCmd;

    final PositionVoltage intakeArmPid;
    final PositionVoltage intakeArmPosVoltCmd;
    

    // SysID
    private SysIdRoutine intakeArmSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,          // Default ramp rate is acceptable
                Volts.of(4),            // Reduce dynamic voltage to 4 to prevent motor brownout
                null,           // Default timeout is acceptable
                                        // Log state with Phoenix SignalLogger class
                (state)->SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts)-> intakeArmMtr.setControl(intakeArmMtrVoltOutCmd.withOutput(volts.in(Volts))),
                null,
                this));

    public SubSys_Intake () {
        // ********************
        // Intake Roller
        // ********************
        // Intake Roller - Motor Config
        intakeRollerMtr.restoreFactoryDefaults();
        intakeRollerMtr.enableVoltageCompensation(12);
        intakeRollerMtr.setInverted(false);
        intakeRollerMtr.setIdleMode(IdleMode.kBrake);
        intakeRollerMtr.setOpenLoopRampRate(IntakeRoller.OpenLoopRampRate);
        intakeRollerMtr.setClosedLoopRampRate(IntakeRoller.PID.ClosedLoopRampRate);
        // Intake Roller - Encoder Config
        intakeRollerMtrEncoder = intakeRollerMtr.getEncoder();
        intakeRollerMtrEncoder.setVelocityConversionFactor(0.016666666666667); // Convert from RPM to Rot/s
        
        // Intake Roller - Motor PID Config
        intakeRollerMtrPID = intakeRollerMtr.getPIDController();
        intakeRollerMtrPID.setP(IntakeRoller.PID.Pgain);
        intakeRollerMtrPID.setI(IntakeRoller.PID.Igain);
        intakeRollerMtrPID.setD(IntakeRoller.PID.Dgain);
        intakeRollerMtrPID.setIZone(IntakeRoller.PID.Izone);
        intakeRollerMtrPID.setFF(IntakeRoller.PID.FFwd);
        intakeRollerMtrPID.setOutputRange(IntakeRoller.PID.MinOutput,IntakeRoller.PID.MaxOutput);

        // ********************
        // Intake Arm
        // ********************
        // Intake Arm Motor Config
        TalonFXConfiguration intakeArmMtrConfiguration = new TalonFXConfiguration();
        intakeArmMtrConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeArmMtrConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeArmMtrConfiguration.Feedback.FeedbackRemoteSensorID = intakeArmCANCoder.getDeviceID();
        intakeArmMtrConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        intakeArmMtrConfiguration.Feedback.RotorToSensorRatio = 1; // 0.005291;

        intakeArmMtrConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ARM_LIMIT_FORWARD;
        intakeArmMtrConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ARM_LIMIT_REVERSE;
        intakeArmMtrConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = ARM_LIMIT_ENABLE;
        intakeArmMtrConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = ARM_LIMIT_ENABLE;     

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
        intakeArmMtrConfiguration.Slot1.kS = 0.25;  // Add 0.25 V output to overcome static friction
        intakeArmMtrConfiguration.Slot1.kV = 0.125; // A velocity target of 1 rps results in 0.12 V output
        intakeArmMtrConfiguration.Slot1.kA = 0.01;  // An acceleration of 1 rps/s requires 0.01 V output
        intakeArmMtrConfiguration.Slot1.kP = 0.1;
        intakeArmMtrConfiguration.Slot1.kI = 0.0;
        intakeArmMtrConfiguration.Slot1.kD = 0.0;

        intakeArmMtrConfiguration.MotionMagic.MotionMagicAcceleration = 0.5; // Target acceleration of 400 rps/s (0.25 seconds to max)
        intakeArmMtrConfiguration.MotionMagic.MotionMagicJerk = 0.5; // Target jerk of 4000 rps/s/s (0.1 seconds)
  

        TalonFXConfigurator intakeArmMtrConfigurator = intakeArmMtr.getConfigurator();
        intakeArmMtrConfigurator.apply(intakeArmMtrConfiguration);

        // Intake Arm CANcoder Config
        CANcoderConfiguration intakeArmCANcoderConfiguration = new CANcoderConfiguration();
        intakeArmCANcoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        intakeArmCANcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        intakeArmCANcoderConfiguration.MagnetSensor.MagnetOffset = SubSys_Intake_Constants.IntakeArm.CANcoderMagOffset;

        CANcoderConfigurator intakeArmCANCoderConfigurator = intakeArmCANCoder.getConfigurator();
        intakeArmCANCoderConfigurator.apply(intakeArmCANcoderConfiguration);

        // Intake Arm Commands Config
        intakeArmMtrVelVoltCmd = new VelocityVoltage(0).withSlot(1); 
        intakeArmMtrVoltMMCmd = new MotionMagicVoltage(0).withSlot(1);     
        intakeArmMtrVelVoltMMCmd = new MotionMagicVelocityVoltage(0).withSlot(1);

        // Intake Arm FF Command
        intakeArmFFVoltCmd = new ArmFeedforward(-0.075, -0.3, -0.05);
        // Gravity -0.3
        // FF 2V = 0.1
        //    3V = 0.125
        //    4V = 0.22
        //    5V = 0.25

        // create a position closed-loop request, voltage output, slot 0 configs
        intakeArmPid = new PositionVoltage(0).withSlot(0);
        intakeArmPosVoltCmd = new PositionVoltage(0).withSlot(1);
        
        // Intake Arm SysID
        /* Speed up signals for better charaterization data */
        BaseStatusSignal.setUpdateFrequencyForAll(250,
            intakeArmMtr.getPosition(),
            intakeArmMtr.getVelocity(),
            intakeArmMtr.getMotorVoltage());

        /* Optimize out the other signals, since they're not particularly helpful for us */
        intakeArmMtr.optimizeBusUtilization();
        
        // Set the logger to log to the first flashdrive plugged in
        SignalLogger.setPath("/media/sda1/");
        SignalLogger.start();
       
    }
    
    @Override
    public void periodic() {
        // ********************
        // Intake Roller
        // ********************
        // Intake Roller - Motor
        SmartDashboard.putNumber("IntakeRollerSpd", intakeRollerMtrEncoder.getVelocity());
        // Intake Roller IR Sensor
        SmartDashboard.putBoolean("IntakeIRRaw", intakeRollerIR.get());

        // ********************
        // Intake Arm
        // ********************
        // Intake Arm Motor
        SmartDashboard.putNumber("IntakeArmPosRaw", intakeArmMtr.getPosition().getValueAsDouble()); // Remove once verified
        SmartDashboard.putNumber("IntakeArmPos", getIntakeArmVelocity());
        SmartDashboard.putNumber("IntakeArmVelRaw", intakeArmMtr.getVelocity().getValueAsDouble()); // Remove once verified
        SmartDashboard.putNumber("IntakeArmVel", getIntakeArmVelocity());
        SmartDashboard.putBoolean("IntakeArmFwdLimitSw", intakeArmMtr.getFault_ForwardHardLimit().getValue());
        SmartDashboard.putBoolean("IntakeArmRevLimitSw", intakeArmMtr.getFault_ReverseHardLimit().getValue());
        SmartDashboard.putBoolean("IntakeArmFwdSwLimit", intakeArmMtr.getFault_ForwardSoftLimit().getValue());
        SmartDashboard.putBoolean("IntakeArmRevSwLimit", intakeArmMtr.getFault_ReverseSoftLimit().getValue());

        //SmartDashboard.putBoolean("Intake /IR Raw value", intakeRollerIR.get());
        //SmartDashboard.putBoolean("Intake/IR Raw value", intakeRollerIR.get());
        //SmartDashboard.putBoolean("Intake/Intake Occupied", getIntakeOccupied());
        //SmartDashboard.putNumber("Intake/ArmEncoderAbsolutePos", intakeArmCANCoder.getAbsolutePosition().getValueAsDouble());
        //SmartDashboard.putNumber("Intake/ArmEncoderPos", intakeArmCANCoder.getPosition().getValueAsDouble());
        //SmartDashboard.putNumber("Intake/ArmPos", getIntakeArmPos());
        //SmartDashboard.putBoolean("Intake/Arm Motor Busy", intakeArmMtrBusy());
        //SmartDashboard.putBoolean("Intake/Arm Motor At Setpoint", intakeArmMtrAtSetpoint());
        //SmartDashboard.putNumber("Intake/Intake Arm PID Position", intakeArmMtr.getClosedLoopReference().getValueAsDouble());
        //SmartDashboard.putBoolean("Intake/ForwardLimitValue", (intakeArmMtr.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround));
        //SmartDashboard.putBoolean("Intake/ReverseLimitValue", (intakeArmMtr.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround));
    }

    // ********************
    // Intake Roller
    // ********************

    /**
     * setIntakeRollerMtrDutyCycle
     * Open Loop Duty Cycle Command 
     * @param dutyCycleCmd double Duty Cycle Cmd (-1 - 1)
     * @return double Intake Roller Motor Velocity (Rot/s)
     */
    public double setIntakeRollerMtrDutyCycle(double dutyCycleCmd){
        intakeRollerMtr.set(dutyCycleCmd);
        return intakeRollerMtrEncoder.getVelocity();
    }

    // Intake Roller IR Sensor
    /** 
     * getIntakeOccupied
     * Get Intake IR Sensor
     * @return true if the intake is occupied with a note
     * */
    public boolean getIntakeOccupied() {
        return !intakeRollerIR.get();
    }

    // ********************
    // Intake Arm
    // ********************
    // Intake Arm Motor

    /**
     * getIntakeArmVelocity
     * Get Intake Arm Velocity
     * @return double Intake Arm Motor Velocity (Rot/s)
     */
    public double getIntakeArmVelocity(){
        return intakeArmMtr.getVelocity().getValueAsDouble();
    }

    /**
    * getIntakeArmPosition
    * Get Intake Arm Velocity
    * @return double Intake Arm Motor Velocity (Rot/s)
    */
    public double getIntakeArmPosition(){
        return intakeArmMtr.getPosition().getValueAsDouble();
    }


    /**
     * setIntakeArmMtrDutyCycle
     * Set Intake Arm Motor Duty Cycle
     * @param dutyCycleCmd double Duty Cycle Cmd (-1 - 1)
     * @return double Intake Arm Motor Velocity (Rot/s)
     */
    public double setIntakeArmMtrDutyCycle(double dutyCycleCmd){
        intakeArmMtr.set(dutyCycleCmd);
        return intakeArmMtr.getVelocity().getValueAsDouble();
    }

    /**
     * setIntakeArmMtrVoltOut
     * Set Intake Arm Motor Voltage Out
     * @param voltCmd double Voltage Command (-12 - 12)
     * @return double Intake Arm Motor Velocity (Rot/s)
     */
    public double setIntakeArmMtrVoltOut(double voltCmd){
        intakeArmMtr.setControl(intakeArmMtrVoltOutCmd.withOutput(voltCmd).withEnableFOC(true));
        //intakeArmMtr.setControl(intakeArmMtrVoltOutCmd.withOutput(
        //    voltCmd+
        //    intakeArmFFVoltCmd.calculate(Units.rotationsToRadians(intakeArmMtr.getPosition().getValueAsDouble()),0.0))
        //    .withEnableFOC(true));
        
        return intakeArmMtr.getVelocity().getValueAsDouble();
    }

    /**
     * setIntakeArmMtrVelVolts
     * Set Intake Arm Motor Velocity in Volt Mode
     * @param spdCmd double Speed Command (Rot/s)
     * @return double Intake Arm Motor Velocity (Rot/s)
     */
    public double setIntakeArmMtrVelVolts(double spdCmd){
        intakeArmMtr.setControl(intakeArmMtrVelVoltCmd.withVelocity(spdCmd).withEnableFOC(true));
        //intakeArmMtr.setControl(intakeArmMtrVelVoltCmd.withVelocity(spdCmd)
        //    .withFeedForward(intakeArmFFVoltCmd.calculate(Units.rotationsToRadians(intakeArmMtr.getPosition().getValueAsDouble()),Units.rotationsToRadians(spdCmd)))
        //    .withEnableFOC(true));
        return intakeArmMtr.getVelocity().getValueAsDouble();
    }

    /**
     * setIntakeArmMtrVelVoltsMM
     * Set Intake Arm Motor Velocity in Volt Mode with Motion Magic
     * @param spdCmd double Speed Command (Rot/s)
     * @return MotionMagicIsRunningValue Motion Magic Running Value
     */
    public MotionMagicIsRunningValue setIntakeArmMtrVelVoltsMM(double spdCmd){
        intakeArmMtr.setControl(intakeArmMtrVelVoltMMCmd.withVelocity(spdCmd).withEnableFOC(true));
        //intakeArmMtr.setControl(intakeArmMtrVelVoltMMCmd.withVelocity(spdCmd)
        //    .withFeedForward(intakeArmFFVoltCmd.calculate(Units.rotationsToRadians(intakeArmMtr.getPosition().getValueAsDouble()),Units.rotationsToRadians(spdCmd)))
        //    .withEnableFOC(true));
        return intakeArmMtr.getMotionMagicIsRunning().getValue();
    }
    
    /**
     * setIntakeArmMtrVoltsMMPos
     * Set Intake Arm Position Command in Volt Mode with Motion Magic
     * @param posCmd double Position Command in Rots
     * @return boolean Stable at Position Command
     */
    public boolean setIntakeArmMtrVoltsMMPos(double posCmd){
        intakeArmMtr.setControl(intakeArmMtrVoltMMCmd.withPosition(posCmd).withEnableFOC(true));
        //intakeArmMtr.setControl(intakeArmMtrVoltMMCmd.withPosition(posCmd)
        //    .withFeedForward(intakeArmFFVoltCmd.calculate(Units.rotationsToRadians(intakeArmMtr.getPosition().getValueAsDouble()),0.0))
        //    .withEnableFOC(true));
        return getIntakeArmAtPos(posCmd);
    }

    /**
     * getIntakeArmAtPos
     * Get Intake Arm at Position Command.  Must be within a position error and velocity error
     * @param posCmd double Position Command in Rots
     * @return boolean At Position Command
     */
    public boolean getIntakeArmAtPos(double posCmd){
        boolean atPos = false;
        double posError = posCmd-intakeArmMtr.getPosition().getValueAsDouble();
        if(Math.abs(posError) < 0.01){
            if(Math.abs(intakeArmMtr.getVelocity().getValueAsDouble())< 0.001){
                atPos = true;
            }
        }
        return atPos;
    }

    // SysID
    public Command intakeArmSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return intakeArmSysIdRoutine.quasistatic(direction);
    }
    public Command intakeArmSysIdDynamic(SysIdRoutine.Direction direction) {
        return intakeArmSysIdRoutine.dynamic(direction);
    }



    /**
     * Calls correct IntakeArm method depending on the values of limit switches
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
     * 
     * @return true if open false if closed
     */
    public boolean getForwardLimit() {
        ForwardLimitValue forwardLimitValue = intakeArmMtr.getForwardLimit().getValue();
        return forwardLimitValue == ForwardLimitValue.Open;
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
    public boolean atLowerLimit(){ 
        return intakeArmMtr.getForwardLimit().getValue().equals(ForwardLimitValue.ClosedToGround);
    }
}
