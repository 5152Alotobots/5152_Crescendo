package frc.robot.library.drivetrains.swerve_ctre.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;

import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Drive_DriveWhileFacingSpeaker extends Command {
    private CommandSwerveDrivetrain subSysSwerve;
    private SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle();
    private DoubleSupplier velocityX;
    private DoubleSupplier velocityY;

    public Cmd_SubSys_Drive_DriveWhileFacingSpeaker(
            CommandSwerveDrivetrain subSysSwerve,
            DoubleSupplier velocityX,
            DoubleSupplier velocityY) {
        this.subSysSwerve = subSysSwerve;
        this.velocityX = velocityX;
        this.velocityY = velocityY;
    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {

            }
        }
    }

    @Override
    public void execute() {
//        Rotation2d targetHeading = AimModule.calculateRobotHeadingAlignShooterToPose()
//        subSysSwerve.applyRequest(() -> drive
//                .withVelocityX(velocityX.getAsDouble())
//                .withVelocityY(velocityY.getAsDouble())
//                .withTargetDirection())
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
