package frc.robot.crescendo.subsystems.bling;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

public class SubSys_Bling_Constants {
    public static final double MAX_LED_BRIGHTNESS = 1;
    public static final int NUM_LEDS = 50;
    public static final CANdle.LEDStripType LED_TYPE = CANdle.LEDStripType.RGBW;

    public static final class Animations {
        public static final ColorFlowAnimation TRANSFER_ANIMATION = new ColorFlowAnimation(255, 145, 0, 0, 1, NUM_LEDS, ColorFlowAnimation.Direction.Forward);
        public static final StrobeAnimation NOTE_IN_INTAKE_ANIMATION = new StrobeAnimation(0, 255, 174, 0, 0.4, NUM_LEDS);
        public static final StrobeAnimation NOTE_IN_SHOOTER_ANIMATION = new StrobeAnimation(0, 179, 255, 0, 0.4, NUM_LEDS);
    }
}
