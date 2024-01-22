package frc.robot.library.drivetrains.mecanum;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SubSys_MecanumDrive extends SubsystemBase {
    private final VictorWrapper frontLeftMotor = new VictorWrapper(1);
    private final VictorWrapper frontRightMotor = new VictorWrapper(2);
    private final VictorWrapper backLeftMotor = new VictorWrapper(3);
    private final VictorWrapper backRightMotor = new VictorWrapper(4);
    private final MecanumDrive robotDrive;
/** Constructs a MecanumDrive and resets the gyro. */
    public SubSys_MecanumDrive() {
        frontLeftMotor.configFactoryDefault();
        frontRightMotor.configFactoryDefault();
        backLeftMotor.configFactoryDefault();
        backRightMotor.configFactoryDefault();

        frontRightMotor.setInverted(true);
        backRightMotor.setInverted(true);

        robotDrive = new MecanumDrive(frontLeftMotor::set, backLeftMotor::set, frontRightMotor::set, backRightMotor::set);
    }

    public void drive(double xspeed, double yspeed, double rot) {
        robotDrive.driveCartesian(xspeed, yspeed, rot);
    }
}
