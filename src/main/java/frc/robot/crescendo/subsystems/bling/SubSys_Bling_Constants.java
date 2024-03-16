package frc.robot.crescendo.subsystems.bling;

import com.ctre.phoenix.led.*;

import java.awt.*;

public class SubSys_Bling_Constants {
    public static final double MAX_LED_BRIGHTNESS = .25;
    public static final int NUM_LEDS = 92;
    public static final int LED_OFFSET = 8;
    public static final boolean DISABLE_STATUS_LED = false;
    public static final CANdle.LEDStripType LED_TYPE = CANdle.LEDStripType.GRB;

    public static final class Animations {
        public static final ColorFlowAnimation SHOOTING_ANIMATION = new ColorFlowAnimation(255, 145, 0, 0, 1, NUM_LEDS, ColorFlowAnimation.Direction.Forward, LED_OFFSET);
        public static final ColorFlowAnimation NO_ALLIANCE_ANIMATION = new ColorFlowAnimation(255, 0, 0, 0, 0.75, NUM_LEDS, ColorFlowAnimation.Direction.Forward, LED_OFFSET);
//        public static final LarsonAnimation NO_ALLIANCE_ANIMATION = new LarsonAnimation(255, 0, 0, 0, 0.5, NUM_LEDS, LarsonAnimation.BounceMode.Center, 20, 8);
    }

    public static final class Colors {
        public static final Color OFF_COLOR = new Color(0, 0, 0);
        public static final Color BLUE_ALLIANCE_COLOR = new Color(0, 0, 255);
        public static final Color RED_ALLIANCE_COLOR = new Color(255, 0, 0);
        public static final Color NO_ALLIANCE_COLOR = new Color(255, 255, 0);
        public static final Color INTAKE_OCCUPIED_COLOR = new Color(0, 255, 0);
        public static final Color SHOOTER_OCCUPIED_COLOR = new Color(140, 48, 255);
        public static final Color SHOOTER_READY_COLOR = new Color(255, 145, 0);
    }
}
