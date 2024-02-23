package frc.robot.crescendo.subsystems.climber;

import edu.wpi.first.math.util.Units;

public class SubSys_Climber_Constants {

    // Climber Position Zero is at the top.  Positive motion will pull the hooks down and the robot up.
    // Max Positions
    public static final double climberLeftMaxHeight = Units.inchesToMeters(30);
    public static final double climberRightMaxHeight = Units.inchesToMeters(30);
    public static final double climberLeftMinHeight = Units.inchesToMeters(12.5);
    public static final double climberRightMinHeight = Units.inchesToMeters(12.0);
    public static final double climberSensorToMechanismRatio = 
      (climberRightMaxHeight-climberRightMinHeight)/(130); //~130 revolutions from max to min
}
