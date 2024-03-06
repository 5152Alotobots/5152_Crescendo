package frc.robot.crescendo.subsystems.shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.DigitalIO_IDs;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.ShooterRoller;
import frc.robot.crescendo.subsystems.shooter.util.ShooterDirection;
import frc.robot.crescendo.subsystems.shooter.util.ShooterIntakeDirection;

import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.AutoAim.*;
import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.PID.Arm.*;
import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.PID.Shooter.*;
import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.ShooterArm.SoftwareLimits.*;
import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.Speeds.*;


/**
 * Handles outputs and inputs from the intake, including rotation motors and limit switches,
 * and Intake spinner.
 */
public class SubSys_Shooter extends SubsystemBase {
    private final TalonFX shooterWheelsMtrRight = new TalonFX(CAN_IDs.ShooterWheelsMtrRight_CAN_ID);
    private final TalonFX shooterWheelsMtrLeft = new TalonFX(CAN_IDs.ShooterWheelsMtrLeft_CAN_ID);
    private final VelocityVoltage shooterVelocity = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
    private final DutyCycleOut shooterDutyCycleOut = new DutyCycleOut(0).withEnableFOC(true);
    private final CANSparkMax shooterRollerMtr = new CANSparkMax(CAN_IDs.ShooterRollerMtr_CAN_ID, MotorType.kBrushless);
    private final SparkPIDController shooterRollerMtrPID;
    private final DigitalInput shooterRollerIR = new DigitalInput(DigitalIO_IDs.ShooterRollerIRDetector_ID);
    private final TalonFX shooterArmMtr = new TalonFX(CAN_IDs.ShooterArmMtr_CAN_ID);
    private final CANcoder shooterArmCANCoder = new CANcoder(CAN_IDs.ShooterArmCANCoder_CAN_ID);

    // PIDs
    final PositionVoltage shooterArmPid;
    final VelocityVoltage shooterWheelsMtrLeftPID;
    final VelocityVoltage shooterWheelsMtrRightPID;
    public SubSys_Shooter () {
        shooterRollerMtr.restoreFactoryDefaults();
        shooterRollerMtr.enableVoltageCompensation(12);
        shooterRollerMtr.setInverted(false);
        shooterRollerMtr.setIdleMode(CANSparkMax.IdleMode.kCoast);
        shooterRollerMtr.setOpenLoopRampRate(ShooterRoller.OpenLoopRampRate);
        shooterRollerMtr.setClosedLoopRampRate(ShooterRoller.PID.ClosedLoopRampRate);

        shooterRollerMtrPID = shooterRollerMtr.getPIDController();
        shooterRollerMtrPID.setP(ShooterRoller.PID.Pgain);
        shooterRollerMtrPID.setI(ShooterRoller.PID.Igain);
        shooterRollerMtrPID.setD(ShooterRoller.PID.Dgain);
        shooterRollerMtrPID.setIZone(ShooterRoller.PID.Izone);
        shooterRollerMtrPID.setFF(ShooterRoller.PID.FFwd);
        shooterRollerMtrPID.setOutputRange(ShooterRoller.PID.MinOutput,ShooterRoller.PID.MaxOutput);
        
        
        TalonFXConfiguration shooterMtrLeftConfiguration = new TalonFXConfiguration();
        TalonFXConfiguration shooterMtrRightConfiguration = new TalonFXConfiguration();
        Slot0Configs shooterSlot0Configs = new Slot0Configs();
        shooterSlot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        shooterSlot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        shooterSlot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        shooterSlot0Configs.kI = 0; // no output for integrated error
        shooterSlot0Configs.kD = 0; // no output for error derivative

        shooterMtrLeftConfiguration.Slot0 = shooterSlot0Configs;
        shooterMtrRightConfiguration.Slot0 = shooterSlot0Configs;

        shooterMtrLeftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMtrRightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooterMtrLeftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterMtrRightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        // Configure Shooter Arm Motor
        TalonFXConfiguration shooterArmMtrConfiguration = new TalonFXConfiguration();
        shooterArmMtrConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shooterArmMtrConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterArmMtrConfiguration.Feedback.FeedbackRemoteSensorID = shooterArmCANCoder.getDeviceID();
        shooterArmMtrConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        shooterArmMtrConfiguration.Slot0.kP = ARM_P;
        shooterArmMtrConfiguration.Slot0.kI = ARM_I;
        shooterArmMtrConfiguration.Slot0.kD = ARM_D;
        shooterArmMtrConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = SHOOTER_LIMIT_FORWARD;
        shooterArmMtrConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = SHOOTER_LIMIT_REVERSE;
        shooterArmMtrConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = SHOOTER_LIMIT_ENABLE;
        shooterArmMtrConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = SHOOTER_LIMIT_ENABLE;

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
     * @param shooterIntakeDirection The {@link ShooterIntakeDirection} to move in
     */
    public void setIntakeOutput(ShooterIntakeDirection shooterIntakeDirection) {
        SmartDashboard.putString("Shooter/Shooter Intake Speed", String.valueOf(shooterIntakeDirection));
        double speed = 0;
        switch (shooterIntakeDirection) {
            case IN:
                if (!getIntakeOccupied()) speed = MAX_INTAKE_SPEED;
                break;
            case TRANSFER:
                if (!getIntakeOccupied()) speed = TRANSFER_INTAKE_SPEED;
                break;
            case OUT:
                speed = -MAX_INTAKE_SPEED;
                break;
            case SHOOT:
                speed = MAX_INTAKE_SPEED;
            default:
                break;
        }
        shooterRollerMtr.set(speed);
    }

    /**
     * @return true when the intake is occupied
     */
    public boolean getIntakeOccupied() {
        return !shooterRollerIR.get();
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
            case AMP_OUT:
                speed = SHOOTER_AMP_SPEED;
                break;
            default:
                break;
        }
        shooterWheelsMtrRight.set(speed);
        shooterWheelsMtrLeft.set(speed);
    }


    /**
     * @param rps Rotations Per Second 1 = 360deg/second, 8 = 360 * 8deg/second
     */
    public void setShooterMotorVelocity(double rps) {
        shooterWheelsMtrLeft.setControl(shooterVelocity.withVelocity(rps));
        shooterWheelsMtrRight.setControl(shooterVelocity.withVelocity(rps));
    }

    /**
     * 
     * @param speed speed from -1 - 1
     */
    public void setShooterDutyCycleOut(double speed) {
        shooterWheelsMtrLeft.setControl(shooterDutyCycleOut.withOutput(speed));
        shooterWheelsMtrRight.setControl(shooterDutyCycleOut.withOutput(speed));
    }

    /**
     * Sets the speed of shooterArmMtr motor output
     * @param percentOutput -1 -> 1
     */
    public void setShooterArmOutput(double percentOutput) {
        shooterArmMtr.set(MathUtil.clamp(percentOutput, -MAX_ARM_ROTATION_SPEED, MAX_ARM_ROTATION_SPEED));
    }

    /**
     * Set the degree of the arm rotation
     * @param degree The degree to rotate to (negative degrees angles away from intake -90 = straight up)
     *               Limited between -180 (forward full) and 0 (back full)
     */
    public void setShooterArmDegree(double degree) {
        SmartDashboard.putNumber("Shooter/Shooter Arm Target Position", degree);
        double limitAdjusted = MathUtil.clamp(degree / 360.0, -180, 0); // Limit to motor limits
        shooterArmMtr.setControl(shooterArmPid.withPosition(limitAdjusted));
    }


    /**
     * @return true if the motors velocity is not zero
     */
    public boolean shooterArmMtrBusy() {
        return (shooterArmMtr.getVelocity().getValueAsDouble() != 0);
    }

    /**
     * @return true if the motor is within position tolerance
     */
    public boolean shooterArmMtrAtSetpoint() {
        return (Math.abs((shooterArmMtr.getPosition().getValueAsDouble() - shooterArmMtr.getClosedLoopReference().getValueAsDouble())) <= SHOOTER_ARM_POSITION_TOLERANCE);
    }


    public void shootDumb(Timer timer) {
        setShooterOutput(ShooterDirection.OUT);
        if (timer.get() > SHOOT_SPIN_UP_TEMP) {
            setIntakeOutput(ShooterIntakeDirection.SHOOT);
        }
    }

    public void shootDumbAtSpeed(Timer timer, double speed) {
        setShooterSpeed(speed);
        if (timer.get() > SHOOT_SPIN_UP_TEMP) {
            setIntakeOutput(ShooterIntakeDirection.SHOOT);
        }
    }

    public void shoot() {
        setShooterSpeed(MAX_SHOOTER_SPPED_MPS);
        if (getShooterWheelsVelocity() >= MAX_SHOOTER_SPPED_MPS - SHOOTER_VELOCITY_TOLERANCE) {
            setIntakeOutput(ShooterIntakeDirection.SHOOT);
        }
    }

    public void shootAtSpeed(double mps) {
        setShooterSpeed(mps);
        if (getShooterWheelsVelocity() >= mps - SHOOTER_VELOCITY_TOLERANCE) {
            setIntakeOutput(ShooterIntakeDirection.SHOOT);
        }
    }

    /**
     * Stops all motors
     */
    public void stopAll() {
        setShooterOutput(ShooterDirection.OFF);
        setIntakeOutput(ShooterIntakeDirection.OFF);
        setShooterArmOutput(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Shooter Arm Velocity", Math.abs(shooterArmMtr.getVelocity().getValueAsDouble()));
        SmartDashboard.putBoolean("Shooter/Shooter Arm Busy", shooterArmMtrBusy());
        SmartDashboard.putNumber("Shooter/Shooter Arm Speed", shooterArmMtr.get());
        SmartDashboard.putString("Shooter/Shooter Intake Speed", String.valueOf(ShooterIntakeDirection.OFF));
        SmartDashboard.putNumber("Shooter/Shooter Speed", 0);
        SmartDashboard.putNumber("Shooter/Shooter PID Target", shooterArmMtr.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putBoolean("Shooter/Shooter At Target", shooterArmMtrAtSetpoint());
        SmartDashboard.putNumber("Shooter/Shooter Wheels Velocity", getShooterWheelsVelocity());
        /* --- PID --- */
        //SmartDashboard.putNumber("Shooter/Shooter Arm Target Position", 0);
        //SmartDashboard.putNumber("Shooter/Shooter Arm Current Position", shooterArmPid.Position);
        /* --- SENSORS --- */
        SmartDashboard.putBoolean("Shooter/Shooter Intake Sensor", getIntakeOccupied());
        SmartDashboard.putNumber("ShooterArmEncoderAbsolutePos", shooterArmCANCoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("ShooterArmEncoderPos", shooterArmCANCoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("ShooterArmMtrPos", shooterArmMtr.getPosition().getValueAsDouble());
        //SmartDashboard.putNumber("ShooterArmPos", getShooterArmPos());
    }
}