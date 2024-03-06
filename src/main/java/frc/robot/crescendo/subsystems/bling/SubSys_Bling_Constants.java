package frc.robot.crescendo.subsystems.bling;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

public class SubSys_Bling_Constants {
    public static final double MAX_LED_BRIGHTNESS = 1;
    public static final int NUM_LEDS = 50;
    public static final CANdle.LEDStripType LED_TYPE = CANdle.LEDStripType.BRG;

    public static final class Animations {
        public static final RainbowAnimation defaultAnimation = new RainbowAnimation(1, 0.5, NUM_LEDS);
        public static final ColorFlowAnimation transferAnimation = new ColorFlowAnimation(255, 145, 0, 0, 1, NUM_LEDS, ColorFlowAnimation.Direction.Forward);
        public static final StrobeAnimation noteInIntake = new StrobeAnimation(0, 255, 174, 0, 0.4, NUM_LEDS);
        public static final StrobeAnimation noteInShooter = new StrobeAnimation(0, 179, 255, 0, 0.4, NUM_LEDS);
    }
}
