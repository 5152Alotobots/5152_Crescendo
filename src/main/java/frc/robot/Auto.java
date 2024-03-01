package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.chargedup.subsystems.arm.SubSys_Arm;
import frc.robot.chargedup.subsystems.bling.SubSys_Bling;
import frc.robot.chargedup.subsystems.hand.SubSys_Hand;
import frc.robot.library.drivetrains.swerve_5152.SubSys_DriveTrain;
import frc.robot.library.gyroscopes.pigeon2.SubSys_PigeonGyro;
import frc.robot.library.vision.photonvision.SubSys_Photonvision;

/**
 * The Auto class is responsible for handling the autonomous mode of the robot. It includes a
 * variety of commands for different tasks such as escaping, state middle leave, state barrier, and
 * others. The class also includes a SendableChooser object to select the desired command.
 * @deprecated Use PathPlanner Inline RobotContainer commands/autos instead
 */
@Deprecated
public class Auto {
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** Constructor for the Auto class. Initializes all the subsystems and commands. */
  public Auto(SubSys_Bling blingSubSys, SubSys_Photonvision photonvisionSubSys, SubSys_Hand handSubSys, SubSys_Arm armSubSys, SubSys_PigeonGyro gyroSubSys, SubSys_DriveTrain driveSubSys) {

  }

  /**
   * Get the selected command from the SendableChooser.
   *
   * @return The selected command.
   */
  public Command getAutoCommand() {
    return m_chooser.getSelected();
  }
}
