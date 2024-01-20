package frc.robot.library.drivetrains.mecanum;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SubSys_MecanumDrive extends SubsystemBase {
    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    private final VictorSPX m_frontLeftMotor = new VictorSPX(1);
    private final VictorSPX m_frontRightMotor = new VictorSPX(2);
    private final VictorSPX m_backLeftMotor = new VictorSPX(3);
    private final VictorSPX m_backRightMotor = new VictorSPX(4);

    private final Encoder m_frontLeftEncoder = new Encoder(0, 1);
    private final Encoder m_frontRightEncoder = new Encoder(2, 3);
    private final Encoder m_backLeftEncoder = new Encoder(4, 5);
    private final Encoder m_backRightEncoder = new Encoder(6, 7);

    private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    private final PIDController m_frontLeftPIDController = new PIDController(1, 0, 0);
    private final PIDController m_frontRightPIDController = new PIDController(1, 0, 0);
    private final PIDController m_backLeftPIDController = new PIDController(1, 0, 0);
    private final PIDController m_backRightPIDController = new PIDController(1, 0, 0);

    private final MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(m_kinematics, m_gyro.getRotation2d(),
            getCurrentDistances());

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

/** Constructs a MecanumDrive and resets the gyro. */
    public SubSys_MecanumDrive() {
        m_frontRightMotor.setInverted(true);
        m_backRightMotor.setInverted(true);
    }

    /**
     * Returns the current state of the drivetrain.
     *
     * @return The current state of the drivetrain.
     */
    public MecanumDriveWheelSpeeds getCurrentState() {
        return new MecanumDriveWheelSpeeds(
                m_frontLeftEncoder.getRate(),
                m_frontRightEncoder.getRate(),
                m_backLeftEncoder.getRate(),
                m_backRightEncoder.getRate());
    }

    /**
     * Returns the current distances measured by the drivetrain.
     *
     * @return The current distances measured by the drivetrain.
     */
    public MecanumDriveWheelPositions getCurrentDistances() {
        return new MecanumDriveWheelPositions(
                m_frontLeftEncoder.getDistance(),
                m_frontRightEncoder.getDistance(),
                m_backLeftEncoder.getDistance(),
                m_backRightEncoder.getDistance());
    }

    /**
     * Set the desired speeds for each wheel.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
        final double frontLeftFeedforward = m_feedforward.calculate(speeds.frontLeftMetersPerSecond);
        final double frontRightFeedforward = m_feedforward.calculate(speeds.frontRightMetersPerSecond);
        final double backLeftFeedforward = m_feedforward.calculate(speeds.rearLeftMetersPerSecond);
        final double backRightFeedforward = m_feedforward.calculate(speeds.rearRightMetersPerSecond);

        final double frontLeftOutput = m_frontLeftPIDController.calculate(
                m_frontLeftEncoder.getRate(), speeds.frontLeftMetersPerSecond);
        final double frontRightOutput = m_frontRightPIDController.calculate(
                m_frontRightEncoder.getRate(), speeds.frontRightMetersPerSecond);
        final double backLeftOutput = m_backLeftPIDController.calculate(
                m_backLeftEncoder.getRate(), speeds.rearLeftMetersPerSecond);
        final double backRightOutput = m_backRightPIDController.calculate(
                m_backRightEncoder.getRate(), speeds.rearRightMetersPerSecond);

        m_frontLeftMotor.setVoltage(frontLeftOutput + frontLeftFeedforward);
        m_frontRightMotor.setVoltage(frontRightOutput + frontRightFeedforward);
        m_backLeftMotor.setVoltage(backLeftOutput + backLeftFeedforward);
        m_backRightMotor.setVoltage(backRightOutput + backRightFeedforward);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(
            double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
        var mecanumDriveWheelSpeeds = m_kinematics.toWheelSpeeds(
                ChassisSpeeds.discretize(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                                : new ChassisSpeeds(xSpeed, ySpeed, rot),
                        periodSeconds));
        mecanumDriveWheelSpeeds.desaturate(kMaxSpeed);
        setSpeeds(mecanumDriveWheelSpeeds);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        m_odometry.update(m_gyro.getRotation2d(), getCurrentDistances());
    }

}
