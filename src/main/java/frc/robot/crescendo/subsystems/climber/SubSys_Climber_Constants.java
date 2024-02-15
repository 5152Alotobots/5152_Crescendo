package frc.robot.crescendo.subsystems.climber;

import edu.wpi.first.math.util.Units;

public class SubSys_Climber_Constants {
    
    // Climber Position Zero is at the top.  Positive motion will pull the hooks down and the robot up.
    // Max Positions
    public static final double climberLeftMaxPos = Units.inchesToMeters(30);
    public static final double climberRightMaxPos = Units.inchesToMeters(30);
    public static final double climberLeftMinPos = Units.inchesToMeters(12.5);
    public static final double climberRightMinPos = Units.inchesToMeters(12.0);
}
