package frc.robot.crescendo.subsystems.shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.crescendo.subsystems.shooter.util.ShooterIntakeDirection;
import frc.robot.crescendo.subsystems.shooter.util.ShooterDirection;

import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.AutoAim.*;
import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.MaxSpeeds.*;
import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.PID.Arm.*;
import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.PID.Shooter.*;


/**
 * Handles outputs and inputs from the intake, including rotation motors and limit switches,
 * and Intake spinner.
 */
public class SubSys_Shooter extends SubsystemBase {
    private final CANSparkMax shooterWheelsMtrRight = new CANSparkMax(CAN_IDs.ShooterWheelsMtrRight_CAN_ID, MotorType.kBrushless);
    private final CANSparkMax shooterWheelsMtrLeft = new CANSparkMax(CAN_IDs.ShooterWheelsMtrLeft_CAN_ID, MotorType.kBrushless);
    private final CANSparkMax shooterRollerMtr = new CANSparkMax(CAN_IDs.ShooterRollerMtr_CAN_ID, MotorType.kBrushless);
    // private final DigitalInput beamSensor = new DigitalInput(0);
    private final TalonFX shooterArmMtr = new TalonFX(CAN_IDs.ShooterArmMtr_CAN_ID);
    private final CANcoder shooterArmCANCoder = new CANcoder(CAN_IDs.ShooterArmCANCoder_CAN_ID);

    // PIDs
    final PositionVoltage shooterArmPid;
    public SubSys_Shooter () {

        shooterRollerMtr.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false);
        // Configure Shooter Arm Motor
        TalonFXConfiguration shooterArmMtrConfiguration = new TalonFXConfiguration();
        shooterArmMtrConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shooterArmMtrConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterArmMtrConfiguration.Feedback.FeedbackRemoteSensorID = shooterArmCANCoder.getDeviceID();
        shooterArmMtrConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        shooterArmMtrConfiguration.Slot0.kP = ARM_P;
        shooterArmMtrConfiguration.Slot0.kI = ARM_I;
        shooterArmMtrConfiguration.Slot0.kD = ARM_D;

        // create a position closed-loop request, voltage output, slot 0 configs
        shooterArmPid = new PositionVoltage(0).withSlot(0);

        TalonFXConfigurator shooterArmMtrConfigurator = shooterArmMtr.getConfigurator();
        shooterArmMtrConfigurator.apply(shooterArmMtrConfiguration);

        // Configure Intake Arm CANcoder
        CANcoderConfiguration shooterArmCaNcoderConfiguration = new CANcoderConfiguration();
        shooterArmCaNcoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        shooterArmCaNcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        shooterArmCaNcoderConfiguration.MagnetSensor.MagnetOffset = SubSys_Shooter_Constants.ShooterArm.CANcoderMagOffset;

        CANcoderConfigurator shooterArmCANCoderConfigurator = shooterArmCANCoder.getConfigurator();
        shooterArmCANCoderConfigurator.apply(shooterArmCaNcoderConfiguration);

        // Shooter motors
        // 1
        shooterWheelsMtrRight.setIdleMode(CANSparkBase.IdleMode.kCoast);
        shooterWheelsMtrRight.getPIDController().setP(SHOOTER_P);
        shooterWheelsMtrRight.getPIDController().setI(SHOOTER_I);
        shooterWheelsMtrRight.getPIDController().setD(SHOOTER_D);
//        shooterWheelsMtr1.getPIDController().setIZone(SHOOTER_IZONE);
        shooterWheelsMtrRight.setClosedLoopRampRate(SHOOTER_RAMP_RATE);

        // 2
        shooterWheelsMtrLeft.setIdleMode(CANSparkBase.IdleMode.kCoast);
        shooterWheelsMtrLeft.getPIDController().setP(SHOOTER_P);
        shooterWheelsMtrLeft.getPIDController().setI(SHOOTER_I);
        shooterWheelsMtrLeft.getPIDController().setD(SHOOTER_D);
//        shooterWheelsMtr1.getPIDController().setIZone(SHOOTER_IZONE);
        shooterWheelsMtrLeft.setClosedLoopRampRate(SHOOTER_RAMP_RATE);
        shooterWheelsMtrLeft.setInverted(true);
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
        shooterWheelsMtrRight.set(speed);
        shooterWheelsMtrLeft.set(speed);
    }

    /**
     * Sets the target speed of the shooter mechanism in meters per second.
     *
     * @param metersPerSecond The desired speed of the shooter mechanism in meters per second.
     */
    public void setShooterSpeed(double metersPerSecond) {
        SmartDashboard.putNumber("Shooter/Shooter Target Speed", metersPerSecond);
        SparkPIDController pidControllerRight = shooterWheelsMtrRight.getPIDController();
        SparkPIDController pidControllerLeft = shooterWheelsMtrLeft.getPIDController();
        double toRpm = metersPerSecond * METERS_TO_RPM_RATIO;
        pidControllerRight.setReference(toRpm, CANSparkBase.ControlType.kVelocity);
        pidControllerLeft.setReference(toRpm, CANSparkBase.ControlType.kVelocity);
    }

    /**
     * Returns the average velocity of the shooter motors.
     * It calculates the average velocity from the velocities obtained from the left and right shooter motor encoders.
     *
     * @return The average velocity of the shooter motors in M/s by the encoder.
     */
    public double getShooterMotorVelocity() {
        RelativeEncoder leftEncoder = shooterWheelsMtrLeft.getEncoder();
        RelativeEncoder rightEncoder = shooterWheelsMtrRight.getEncoder();
        double averageRpm = (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2.0;
        return averageRpm / METERS_TO_RPM_RATIO;
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
     * @return true if the motors velocity does not exceed SHOOTER_ARM_VELOCITY_TOLERANCE
     */
    public boolean shooterArmMtrBusy() {
        return (Math.abs(shooterArmMtr.getVelocity().getValueAsDouble()) <= SHOOTER_ARM_VELOCITY_TOLERANCE);
    }


    public void shootDumb(Timer timer) {
        setShooterOutput(ShooterDirection.OUT);
        if (timer.get() > SHOOT_SPIN_UP_TEMP) {
            setIntakeOutput(ShooterIntakeDirection.IN);
        }
    }

    public void shoot() {
        setShooterSpeed(MAX_SHOOTER_SPPED_MPS);
        if (getShooterMotorVelocity() >= MAX_SHOOTER_SPPED_MPS - SHOOTER_VELOCITY_TOLERANCE) {
            setIntakeOutput(ShooterIntakeDirection.IN);
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
        SmartDashboard.putNumber("Shooter/Shooter Arm Speed", shooterArmMtr.get());
        SmartDashboard.putString("Shooter/Shooter Intake Speed", String.valueOf(ShooterIntakeDirection.OFF));
        SmartDashboard.putNumber("Shooter/Shooter Speed", 0);
        /* --- PID --- */
        SmartDashboard.putNumber("Shooter/Shooter Arm Target Position", 0);
        SmartDashboard.putNumber("Shooter/Shooter Arm Current Position", shooterArmPid.Position);
        /* --- SENSORS --- */
        SmartDashboard.putBoolean("Shooter/Shooter Intake Sensor", getIntakeOccupied());

    }
}