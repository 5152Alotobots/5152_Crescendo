package frc.robot.crescendo.subsystems.shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.IntakeArm;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.IntakeRoller;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.ShooterArm;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.ShooterRoller;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.ShooterWheels;
import frc.robot.crescendo.subsystems.shooter.util.IntakeDirection;
import frc.robot.crescendo.subsystems.shooter.util.ShooterDirection;

import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.AutoAim.SHOOT_SPIN_UP_TEMP;
import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.MaxSpeeds.*;


/**
 * Handles outputs and inputs from the intake, including rotation motors and limit switches,
 * and Intake spinner.
 */
public class SubSys_Shooter extends SubsystemBase {
    
    // Shooter Wheels   
    private final CANSparkMax shooterWheelsMtr1 = new CANSparkMax(CAN_IDs.ShooterWheelsMtrRight_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final SparkPIDController shooterWheelsMtr1PID;
    private final RelativeEncoder shooterWheelsMtr1Encoder;
    private final CANSparkMax shooterWheelsMtr2 = new CANSparkMax(CAN_IDs.ShooterWheelsMtrLeft_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final SparkPIDController shooterWheelsMtr2PID;
    private final RelativeEncoder shooterWheelsMtr2Encoder;

    // Shooter Rollers
    private final CANSparkMax shooterRollerMtr = new CANSparkMax(CAN_IDs.ShooterRollerMtr_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final SparkPIDController shooterRollerMtrPID;
    private final RelativeEncoder shooterRollerMtrEncoder;

    private final TalonFX shooterArmMtr = new TalonFX(CAN_IDs.ShooterArmMtr_CAN_ID);
    private final CANcoder shooterArmCANCoder = new CANcoder(CAN_IDs.ShooterArmCANCoder_CAN_ID);

    // PIDs
    final PositionVoltage shooterArmPid;
    public SubSys_Shooter () {

        // Shooter Wheels 
        shooterWheelsMtr1.restoreFactoryDefaults();
        shooterWheelsMtr1.enableVoltageCompensation(12);
        shooterWheelsMtr1.setInverted(false);
        shooterWheelsMtr1.setIdleMode(CANSparkMax.IdleMode.kCoast);
        shooterWheelsMtr1.setOpenLoopRampRate(ShooterWheels.OpenLoopRampRate);
        shooterWheelsMtr1.setClosedLoopRampRate(ShooterWheels.PID.ClosedLoopRampRate);

        shooterWheelsMtr1Encoder = shooterWheelsMtr1.getEncoder();

        shooterWheelsMtr1PID = shooterWheelsMtr1.getPIDController();
        shooterWheelsMtr1PID.setP(ShooterWheels.PID.Pgain);
        shooterWheelsMtr1PID.setI(ShooterWheels.PID.Igain);
        shooterWheelsMtr1PID.setD(ShooterWheels.PID.Dgain);
        shooterWheelsMtr1PID.setIZone(ShooterWheels.PID.Izone);
        shooterWheelsMtr1PID.setFF(ShooterWheels.PID.FFwd);
        shooterWheelsMtr1PID.setOutputRange(ShooterWheels.PID.MinOutput,ShooterWheels.PID.MaxOutput);

        shooterWheelsMtr2.restoreFactoryDefaults();
        shooterWheelsMtr2.enableVoltageCompensation(12);
        shooterWheelsMtr2.setInverted(true);
        shooterWheelsMtr2.setIdleMode(CANSparkMax.IdleMode.kCoast);
        shooterWheelsMtr2.setOpenLoopRampRate(ShooterWheels.OpenLoopRampRate);
        shooterWheelsMtr2.setClosedLoopRampRate(ShooterWheels.PID.ClosedLoopRampRate);

        shooterWheelsMtr2Encoder = shooterWheelsMtr2.getEncoder();

        shooterWheelsMtr2PID = shooterWheelsMtr2.getPIDController();
        shooterWheelsMtr2PID.setP(ShooterWheels.PID.Pgain);
        shooterWheelsMtr2PID.setI(ShooterWheels.PID.Igain);
        shooterWheelsMtr2PID.setD(ShooterWheels.PID.Dgain);
        shooterWheelsMtr2PID.setIZone(ShooterWheels.PID.Izone);
        shooterWheelsMtr2PID.setFF(ShooterWheels.PID.FFwd);
        shooterWheelsMtr2PID.setOutputRange(ShooterWheels.PID.MinOutput,ShooterWheels.PID.MaxOutput);
      
        // Shooter Rollers
        shooterRollerMtr.restoreFactoryDefaults();
        shooterRollerMtr.enableVoltageCompensation(12);
        shooterRollerMtr.setInverted(false);
        shooterRollerMtr.setIdleMode(CANSparkMax.IdleMode.kCoast);
        shooterRollerMtr.setOpenLoopRampRate(ShooterRoller.OpenLoopRampRate);
        shooterRollerMtr.setClosedLoopRampRate(ShooterRoller.PID.ClosedLoopRampRate);
        shooterRollerMtr.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false);

        shooterRollerMtrEncoder = shooterRollerMtr.getEncoder();

        shooterRollerMtrPID = shooterRollerMtr.getPIDController();
        shooterRollerMtrPID.setP(ShooterRoller.PID.Pgain);
        shooterRollerMtrPID.setI(ShooterRoller.PID.Igain);
        shooterRollerMtrPID.setD(ShooterRoller.PID.Dgain);
        shooterRollerMtrPID.setIZone(ShooterRoller.PID.Izone);
        shooterRollerMtrPID.setFF(ShooterRoller.PID.FFwd);
        shooterRollerMtrPID.setOutputRange(ShooterRoller.PID.MinOutput,ShooterRoller.PID.MaxOutput);
        

        // ***** Configure Shooter Arm Motor *****
        TalonFXConfiguration shooterArmMtrConfiguration = new TalonFXConfiguration();
        shooterArmMtrConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shooterArmMtrConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterArmMtrConfiguration.Feedback.FeedbackRemoteSensorID = shooterArmCANCoder.getDeviceID();
        shooterArmMtrConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        shooterArmMtrConfiguration.Slot0.kP = 1.5;
        shooterArmMtrConfiguration.Slot0.kI = 0.4;
        shooterArmMtrConfiguration.Slot0.kD = 0;

        // Limits
        shooterArmMtrConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        shooterArmMtrConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ShooterArm.FwdSWLimitPos;
        shooterArmMtrConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        shooterArmMtrConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ShooterArm.RevSWLimitPos;
        
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

    /**
     * Sets the Intake output
     *
     * @param intakeDirection The {@link IntakeDirection} to move in
     */
    public void setIntakeOutput(IntakeDirection intakeDirection) {
        SmartDashboard.putString("Shooter/Shooter Intake Speed", String.valueOf(intakeDirection));
        double speed = 0;
        switch (intakeDirection) {
            case IN:
                speed = MAX_INTAKE_SPEED;
                break;
            case TRANSFER:
                speed = TRANSFER_INTAKE_SPEED;
                break;
            case OUT:
                speed = -MAX_INTAKE_SPEED;
                break;
            default:
                break;
        }
        shooterRollerMtr.set(speed);
    }

    /**
     * @return true when the intake is occupied
     */
    public boolean getIntakeOccupied() {
        return shooterRollerMtr.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    }

    // Intake ------

    /**
     * Set direction of shooter motor output
     * @param shooterDirection The {@link ShooterDirection} to run the motor at
     */
    public void setShooterOutput(ShooterDirection shooterDirection) {
        SmartDashboard.putString("Shooter/Shooter Speed", shooterDirection.toString());
        double speed = 0;
        switch (shooterDirection) {
            case IN:
                speed = -MAX_SHOOTER_SPEED;
                break;
            case OUT:
                speed = MAX_SHOOTER_SPEED;
                break;
            default:
                break;
        }
        shooterWheelsMtr1.set(speed);
        shooterWheelsMtr2.set(speed);
    }


    // Arm ------

    /**
     * Sets the speed of shooterArmMtr motor output
     * @param percentOutput -1 -> 1
     */
    public void setShooterArmOutput(double percentOutput) {
        shooterArmMtr.set(MathUtil.clamp(percentOutput, -MAX_ARM_ROTATION_SPEED, MAX_ARM_ROTATION_SPEED));
    }

    /**
     * Set the degree of the arm rotation
     * @param degree The degree to rotate to
     */
    public void setShooterArmDegree(double degree) {
        SmartDashboard.putNumber("Shooter/Shooter Arm Target Position", degree);
        shooterArmMtr.setControl(shooterArmPid.withPosition(degree / 360.0));
    }


    /**
     * @return true if the motors velocity does not equal 0
     */
    public boolean shooterArmMtrBusy() {
        return (shooterArmMtr.getVelocity().getValueAsDouble() != 0);
    }


    public void shootDumb(Timer timer) {
        setShooterOutput(ShooterDirection.OUT);
        if (timer.get() > SHOOT_SPIN_UP_TEMP) {
            setIntakeOutput(IntakeDirection.IN);
        }
    }

    public void shoot() {
        setShooterOutput(ShooterDirection.OUT);
        setIntakeOutput(IntakeDirection.IN);
    }

    /**
     * Stops all motors
     */
    public void stopAll() {
        setShooterOutput(ShooterDirection.OFF);
        setIntakeOutput(IntakeDirection.OFF);
        setShooterArmOutput(0);
    }

    public boolean setShootSpd(double spdCmd){
        return true;
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Shooter/Shooter Arm Speed", shooterArmMtr.get());
        //SmartDashboard.putString("Shooter/Shooter Intake Speed", String.valueOf(IntakeDirection.OFF));
        //SmartDashboard.putNumber("Shooter/Shooter Speed", 0);
        /* --- PID --- */
        //SmartDashboard.putNumber("Shooter/Shooter Arm Target Position", 0);
        //SmartDashboard.putNumber("Shooter/Shooter Arm Current Position", shooterArmPid.Position);
        /* --- SENSORS --- */
        //SmartDashboard.putBoolean("Shooter/Shooter Intake Sensor", getIntakeOccupied());
        SmartDashboard.putNumber("ShooterArmEncoderAbsolutePos", shooterArmCANCoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("ShooterArmEncoderPos", shooterArmCANCoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("ShooterArmMtrPos", shooterArmMtr.getPosition().getValueAsDouble());
        
        SmartDashboard.putNumber("ShooterArmPos", getShooterArmPos());
        SmartDashboard.putNumber("ShooterArmPosDeg", getShooterArmPosDeg());
    }

    // New Shooter Methods

    // ***** Shooter Arm *****
    public double getShooterArmPos(){
        return shooterArmMtr.getPosition().getValueAsDouble();
    }

    public double getShooterArmPosDeg(){
        return getShooterArmPos()*360;
    }

    public void setShooterArmDutyCycleCmd(double dcCmd){
        shooterArmMtr.set(dcCmd);
    }

    public boolean setShooterArmPosCmd(double posCmdRev){
        // Position in Revolutions
        boolean atPos = false;
        double error = posCmdRev-getShooterArmPos();
        if (error > 0.05){
            setShooterArmDutyCycleCmd(ShooterArm.ShooterArmPosCmdFastDutyCycle);
            atPos = false;
        } else if(error > 0.015){
            setShooterArmDutyCycleCmd(ShooterArm.ShooterArmPosCmdSlowDutyCycle);
            atPos = false;
        } else if(error < -0.05){
            setShooterArmDutyCycleCmd(-1*ShooterArm.ShooterArmPosCmdFastDutyCycle);
            atPos = false;
        }else if(error < -0.015){
            setShooterArmDutyCycleCmd(-1*ShooterArm.ShooterArmPosCmdSlowDutyCycle);
            atPos = false;
        }else {
            setShooterArmDutyCycleCmd(0.0);
            atPos = true;
        }
        return atPos;
    }

    public boolean setShooterArmPosCmdDeg(double posCmdDeg){
        // Position in Degrees
        return setShooterArmPosCmd(posCmdDeg/360);
    }

    // ***** Shooter Rollers *****
    
    public void setShooterRollerDutyCycleCmd(double dcCmd){
        shooterRollerMtr.set(dcCmd);
    }

    public void transferNote(){
        if (getIntakeOccupied()){
            setShooterRollerDutyCycleCmd(0);
        } else {
            setShooterRollerDutyCycleCmd(ShooterRoller.TransferNoteDutyCycle);
        }
    }

    public void ejectNoteRollers(){
        setShooterRollerDutyCycleCmd(ShooterRoller.EjectNoteDutyCycle);
    }

    public void shootNoteRollers(){
        setShooterRollerDutyCycleCmd(ShooterRoller.ShootNoteDutyCycle);
    }

    public void sourceIntakeNoteRollers(){
        setShooterRollerDutyCycleCmd(ShooterRoller.SourceIntakeNoteDutyCycle);
    }

    // Shooter Wheels

    public void setShooterWheelsDutyCycleCmd(double dcCmd){
        shooterWheelsMtr1.set(dcCmd);
        shooterWheelsMtr2.set(dcCmd);
    }

    public void shootSpeakerShooterWheels(){
        setShooterWheelsDutyCycleCmd(ShooterWheels.ShootSpeakerDutyCycleCmd);
    }

    public void shootAmpShooterWheels(){
        setShooterWheelsDutyCycleCmd(ShooterWheels.ShootAmpDutyCycleCmd);
    }

    public void intakeNoteShooterWheels(){
        setShooterWheelsDutyCycleCmd(ShooterWheels.IntakeNoteDutyCycleCmd);
    }
 
}