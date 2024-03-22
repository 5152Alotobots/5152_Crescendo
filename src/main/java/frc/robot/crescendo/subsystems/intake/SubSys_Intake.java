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
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.DigitalIO_IDs;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.IntakeArm;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.IntakeRollerMtr;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.IntakeArm.IntakeArmMtr;

import static edu.wpi.first.units.Units.Seconds;
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
    final PositionVoltage intakeArmMtrPosVoltCmd;
    

    // SysID
    private SysIdRoutine intakeRollerSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.1).per(Seconds.of(1)),          // Default ramp rate is acceptable
                Volts.of(4),  // Reduce dynamic voltage to 4 to prevent motor brownout
                Seconds.of(1),           // Default timeout is acceptable
                                        // Log state with Phoenix SignalLogger class
                (state)->SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts)-> intakeRollerMtr.setVoltage(volts.in(Volts)),
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
        intakeRollerMtr.setOpenLoopRampRate(IntakeRollerMtr.OpenLoopRampRate);
        intakeRollerMtr.setClosedLoopRampRate(IntakeRollerMtr.PID.ClosedLoopRampRate);

        // Intake Roller - Encoder Config
        intakeRollerMtrEncoder = intakeRollerMtr.getEncoder();
        intakeRollerMtrEncoder.setVelocityConversionFactor(0.016666666666667); // Convert from RPM to Rot/s
        
        // Intake Roller - Motor PID Config
        intakeRollerMtrPID = intakeRollerMtr.getPIDController();
        intakeRollerMtrPID.setP(IntakeRollerMtr.PID.Pgain);
        intakeRollerMtrPID.setI(IntakeRollerMtr.PID.Igain);
        intakeRollerMtrPID.setD(IntakeRollerMtr.PID.Dgain);
        intakeRollerMtrPID.setIZone(IntakeRollerMtr.PID.Izone);
        intakeRollerMtrPID.setFF(IntakeRollerMtr.PID.FFwd);
        intakeRollerMtrPID.setOutputRange(IntakeRollerMtr.PID.MinOutput,IntakeRollerMtr.PID.MaxOutput);
                
        // ********************
        // Intake Arm
        // ********************
        // Intake Arm Motor Config           

        TalonFXConfigurator intakeArmMtrConfigurator = intakeArmMtr.getConfigurator();
        intakeArmMtrConfigurator.apply(IntakeArm.IntakeArmMtr.intakeArmMtrConfiguration.Feedback.withFeedbackRemoteSensorID(intakeArmCANCoder.getDeviceID()));


        // Intake Arm CANcoder Config
        CANcoderConfigurator intakeArmCANCoderConfigurator = intakeArmCANCoder.getConfigurator();
        intakeArmCANCoderConfigurator.apply(IntakeArm.IntakeArmCANCoder.canCoderConfiguration);

        // Intake Arm Commands Config
        intakeArmMtrVelVoltCmd = new VelocityVoltage(0).withSlot(0); 
        intakeArmMtrVoltMMCmd = new MotionMagicVoltage(0).withSlot(0);     
        intakeArmMtrVelVoltMMCmd = new MotionMagicVelocityVoltage(0).withSlot(0);
        intakeArmMtrPosVoltCmd = new PositionVoltage(0).withSlot(1);

        // Intake Arm FF Command
        intakeArmFFVoltCmd = new ArmFeedforward(
            IntakeArmMtr.slot0Configs.kS,    
            IntakeArmMtr.slot0Configs.kG,
            IntakeArmMtr.slot0Configs.kV,
            IntakeArmMtr.slot0Configs.kA);

        // create a position closed-loop request, voltage output, slot 0 configs
        intakeArmPid = new PositionVoltage(0).withSlot(0);
        
        
        // Intake Arm SysID
        /* Speed up signals for better charaterization data */
        BaseStatusSignal.setUpdateFrequencyForAll(250,
            intakeArmMtr.getPosition(),
            intakeArmMtr.getVelocity(),
            intakeArmMtr.getMotorVoltage());

        /* Optimize out the other signals, since they're not particularly helpful for us */
        intakeArmMtr.optimizeBusUtilization();
        
        // Set the logger to log to the first flashdrive plugged in
        //SignalLogger.setPath("/media/sda1/");
        //SignalLogger.start();
       
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
        SmartDashboard.putNumber("IntakeArmPos", getIntakeArmPosition());
        SmartDashboard.putNumber("IntakeArmVelRaw", intakeArmMtr.getVelocity().getValueAsDouble()); // Remove once verified
        SmartDashboard.putNumber("IntakeArmVel", getIntakeArmVelocity());

        //SmartDashboard.putBoolean("IntakeArmFwdLimitSw", intakeArmMtr.getFault_ForwardHardLimit().getValue());
        //SmartDashboard.putBoolean("IntakeArmRevLimitSw", intakeArmMtr.getFault_ReverseHardLimit().getValue());
        //SmartDashboard.putBoolean("IntakeArmFwdSwLimit", intakeArmMtr.getFault_ForwardSoftLimit().getValue());
        //SmartDashboard.putBoolean("IntakeArmRevSwLimit", intakeArmMtr.getFault_ReverseSoftLimit().getValue());

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

    public void setIntakeRollerMtrStop(){
        intakeRollerMtr.stopMotor();
    }

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

    public void setIntakeArmMotorStop(){
        intakeArmMtr.stopMotor();
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
     * setIntakeArmMtrPosVolts
     * Set Intake Arm Position Command in Volt Mode with Motion Magic
     * @param posCmd double Position Command in Rots
     * @return boolean Stable at Position Command
     */
    public boolean setIntakeArmMtrPosVolts(double posCmd){
        intakeArmMtr.setControl(intakeArmMtrPosVoltCmd.withPosition(posCmd).withEnableFOC(true));
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
        if(Math.abs(posError) < IntakeArm.ErrorTolerances.PositionErrorTolerance){
            if(Math.abs(intakeArmMtr.getVelocity().getValueAsDouble())< IntakeArm.ErrorTolerances.VelocityErrorTolerance){
                atPos = true;
            }
        }
        SmartDashboard.putBoolean("IntakeArmAtPos", atPos);
        return atPos;
    }

    // SysID
    public Command intakeRollerSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return intakeRollerSysIdRoutine.quasistatic(direction);
    }
    public Command intakeRollerSysIdDynamic(SysIdRoutine.Direction direction) {
        return intakeRollerSysIdRoutine.dynamic(direction);
    }
}
