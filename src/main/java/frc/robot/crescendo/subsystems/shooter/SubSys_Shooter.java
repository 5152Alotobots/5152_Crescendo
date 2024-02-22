package frc.robot.crescendo.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.crescendo.subsystems.shooter.util.IntakeDirection;
import frc.robot.crescendo.subsystems.shooter.util.ShooterDirection;

import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.AutoAim.SHOOT_SPIN_UP_TEMP;
import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.MaxSpeeds.*;


/**
 * Handles outputs and inputs from the intake, including rotation motors and limit switches,
 * and Intake spinner.
 */
public class SubSys_Shooter extends SubsystemBase {
    private final TalonSRX shooterWheelsMtr1 = new TalonSRX(CAN_IDs.ShooterWheelsMtrRight_CAN_ID);
    private final TalonSRX shooterWheelsMtr2 = new TalonSRX(CAN_IDs.ShooterWheelsMtrLeft_CAN_ID);
    private final CANSparkMax shooterRollerMtr = new CANSparkMax(CAN_IDs.ShooterRollerMtr_CAN_ID, MotorType.kBrushless);
    // private final DigitalInput beamSensor = new DigitalInput(0);
    private final TalonFX shooterArmMtr = new TalonFX(CAN_IDs.ShooterArmMtr_CAN_ID);
    private final CANcoder shooterArmCANCoder = new CANcoder(CAN_IDs.ShooterArmCANCoder_CAN_ID);

    // PIDs
    final PositionVoltage shooterArmPid;
    public SubSys_Shooter () {

        shooterRollerMtr.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false);
        // Configure Shooter Arm Motor
        TalonFXConfiguration shooterArmMtrConfiguration = new TalonFXConfiguration();
        shooterArmMtrConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shooterArmMtrConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterArmMtrConfiguration.Feedback.FeedbackRemoteSensorID = shooterArmCANCoder.getDeviceID();
        shooterArmMtrConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        shooterArmMtrConfiguration.Slot0.kP = 1.5;
        shooterArmMtrConfiguration.Slot0.kI = 0.4;
        shooterArmMtrConfiguration.Slot0.kD = 0;

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
        return shooterRollerMtr.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
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
        shooterWheelsMtr1.set(TalonSRXControlMode.PercentOutput, speed);
        shooterWheelsMtr2.set(TalonSRXControlMode.PercentOutput, speed);
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
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Shooter Arm Speed", shooterArmMtr.get());
        SmartDashboard.putString("Shooter/Shooter Intake Speed", String.valueOf(IntakeDirection.OFF));
        SmartDashboard.putNumber("Shooter/Shooter Speed", 0);
        /* --- PID --- */
        SmartDashboard.putNumber("Shooter/Shooter Arm Target Position", 0);
        SmartDashboard.putNumber("Shooter/Shooter Arm Current Position", shooterArmPid.Position);
        /* --- SENSORS --- */
        SmartDashboard.putBoolean("Shooter/Shooter Intake Sensor", getIntakeOccupied());

    }
}