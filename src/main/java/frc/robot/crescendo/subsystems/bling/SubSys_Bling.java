package frc.robot.crescendo.subsystems.bling;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.awt.*;

import static frc.robot.Constants.CAN_IDs.CANDLE_CAN_ID;
import static frc.robot.crescendo.subsystems.bling.SubSys_Bling_Constants.Animations.NO_ALLIANCE_ANIMATION;
import static frc.robot.crescendo.subsystems.bling.SubSys_Bling_Constants.Colors.*;
import static frc.robot.crescendo.subsystems.bling.SubSys_Bling_Constants.*;

public class SubSys_Bling extends SubsystemBase {
    private Animation currentAnimation;
    private Animation queuedAnimation;
    private Color currentSolidColor;
    private Color queuedColor;
    private final CANdle controller = new CANdle(CANDLE_CAN_ID);

    public SubSys_Bling() {
        controller.configBrightnessScalar(MAX_LED_BRIGHTNESS);
        controller.configLEDType(LED_TYPE);
        controller.configStatusLedState(DISABLE_STATUS_LED);
    }


    /**
     * Sets the LED strip to be the color of the alliance reported by the FMS/DS
     */
    public void setLedToAllianceColor() {
        clearAnimation();
        // Alliance colors
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
                setSolidColor(RED_ALLIANCE_COLOR);
            } else {
                setSolidColor(BLUE_ALLIANCE_COLOR);
            }
        } else {
            runAnimation(NO_ALLIANCE_ANIMATION);
        }


    }

    /**
     * Sets the color of the LED strip. Overrides previously running animations
     *
     */
    public void setSolidColor(Color color) {
        clearAnimation();
        currentSolidColor = color;
    }

    /**
     * Clears the LEDs solid color (Turns off LEDs)
     */
    public void clearSolidColor() {
        currentSolidColor = OFF_COLOR;
    }

    /**
     * Queues a color and doesn't set the next one until released
     *
     * @param toQueue The color to queue
     */
    public void queueColor(Color toQueue) {
        queuedColor = toQueue;
    }

    /**
     * Sets the next color if available. If not, runs default behavior
     */
    public void setQueuedColor() {
        if (queuedColor != null) {
            setSolidColor(queuedColor);
            queuedColor = null;
        } else {
            runDefault();
        }
    }

    /**
     * Sets the CANdle and attached LED's animation. Overrides previous solid colors and other animations
     *
     * @param animation The animation to set
     */
    public void runAnimation(Animation animation) {
        clearAnimation();
        currentAnimation = animation;
    }

    /**
     * Clears the current animation
     */
    public void clearAnimation() {
        controller.clearAnimation(0);
        currentAnimation = null;
    }

    /**
     * Queues an animation and doesn't run the next one until released
     *
     * @param toQueue The animation to queue
     */
    public void queueAnimation(Animation toQueue) {
        queuedAnimation = toQueue;
    }

    /**
     * Runs the next animation if available. If not, runs default behavior
     */
    public void runQueuedAnimation() {
        if (queuedAnimation != null) {
            runAnimation(queuedAnimation);
            queuedAnimation = null;
        } else {
            runDefault();
        }
    }

    /**
     * Clears all settings (turns off LEDs)
     */
    public void clearAll() {
        clearAnimation();
        clearSolidColor();
    }

    /**
     * Runs the default action
     */
    public void runDefault() {
        setLedToAllianceColor();
    }


    /**
     * Updates the controller with the current state.
     * Should be run in the periodic section of the command
     */
    public void update() {
        if (currentAnimation == null) {
            controller.clearAnimation(0);
            controller.setLEDs(currentSolidColor.getRed(), currentSolidColor.getGreen(), currentSolidColor.getBlue(), 0, LED_OFFSET, NUM_LEDS);
        } else {
            controller.animate(currentAnimation, 0);
        }
    }

    @Override
    public void periodic() {
    }

}
