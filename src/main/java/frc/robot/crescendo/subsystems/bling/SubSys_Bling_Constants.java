package frc.robot.crescendo.subsystems.bling;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;

import java.awt.*;

public class SubSys_Bling_Constants {
    public static final double MAX_LED_BRIGHTNESS = 1;
    public static final int NUM_LEDS = 200;
    public static final CANdle.LEDStripType LED_TYPE = CANdle.LEDStripType.GRB;

    public static final class Animations {
        public static final ColorFlowAnimation SHOOTING_ANIMATION = new ColorFlowAnimation(255, 145, 0, 0, 0.03, NUM_LEDS, ColorFlowAnimation.Direction.Forward);
    }

    public static final class Colors {
        public static final Color OFF_COLOR = new Color(0, 0, 0);
        public static final Color BLUE_ALLIANCE_COLOR = new Color(0, 0, 255);
        public static final Color RED_ALLIANCE_COLOR = new Color(255, 0, 0);
        public static final Color INTAKE_OCCUPIED_COLOR = new Color(78, 255, 96);
        public static final Color SHOOTER_OCCUPIED_COLOR = new Color(188, 48, 255);
    }
}
