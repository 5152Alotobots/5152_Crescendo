package frc.robot.Library.DriveTrains.SwerveDrive.Cmds;
import com.fasterxml.jackson.databind.deser.std.UntypedObjectDeserializer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Library.DriveTrains.SwerveDrive.SubSys_SwerveDrive;
import swervelib.SwerveController;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopDrive extends Command {
    /** Creates a new TeleopDrive. */
    private final SubSys_SwerveDrive swerveDrive;
    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier omega;
    private final BooleanSupplier fieldRelative;
    private final SwerveController swerveController;
    public TeleopDrive(SubSys_SwerveDrive swerveDrive, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega, BooleanSupplier fieldRelative) {
        this.fieldRelative = fieldRelative;
        this.swerveController = swerveDrive.getSwerveController();
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.swerveDrive = swerveDrive;

        addRequirements(swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double xVelocity = Math.pow(vX.getAsDouble(), 3);
        double yVelocity = Math.pow(vY.getAsDouble(), 3);
        double angVelocity = Math.pow(omega.getAsDouble(), 3);
        SmartDashboard.putNumber("xVelocity", xVelocity);
        SmartDashboard.putNumber("yVelocity", yVelocity);
        SmartDashboard.putNumber("angVelocity", angVelocity);

        swerveDrive.drive(
                new Translation2d(
                        xVelocity * swerveController.config.maxSpeed,
                        yVelocity * Units.feetToMeters(14.5)),
                angVelocity * swerveController.config.maxAngularVelocity,
                fieldRelative.getAsBoolean()
        );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
