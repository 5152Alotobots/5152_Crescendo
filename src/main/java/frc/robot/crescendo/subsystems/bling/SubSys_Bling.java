package frc.robot.crescendo.subsystems.bling;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CAN_IDs.CANDLE_CAN_ID;
import static frc.robot.crescendo.subsystems.bling.SubSys_Bling_Constants.LED_TYPE;
import static frc.robot.crescendo.subsystems.bling.SubSys_Bling_Constants.MAX_LED_BRIGHTNESS;

public class SubSys_Bling extends SubsystemBase {
    private Animation currentAnimation;
    private boolean animationMode;
    private final CANdle controller = new CANdle(CANDLE_CAN_ID);

    public SubSys_Bling() {
        controller.configBrightnessScalar(MAX_LED_BRIGHTNESS);
        controller.configLEDType(LED_TYPE);
    }


    /**
     * Sets the LED strip to be the color of the alliance reported by the FMS/DS
     */
    public void setLedToAllianceColor() {
        // Alliance colors
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
                controller.setLEDs(255, 0, 0);
            }
        } else {
            controller.setLEDs(0, 0, 255);
        }
    }

    /**
     * Sets the color of the LED strip. Overrides previously running animations
     *
     * @param r (0-255)
     * @param g (0-255)
     * @param b (0-255)
     */
    public void setSolidColor(int r, int g, int b) {
        animationMode = false;
        controller.setLEDs(r, g, b);
    }

    /**
     * Sets the CANdle and attached LED's animation. Overrides previous solid colors
     *
     * @param animation The animation to set
     */
    public void setAnimation(Animation animation) {
        animationMode = true;
        currentAnimation = animation;
    }

    /**
     * Clears the current animation
     */
    public void clearAnimation() {
        animationMode = false;
        currentAnimation = null;
    }


    @Override
    public void periodic() {
        // If we have no input
        if (currentAnimation == null && animationMode) {
            setLedToAllianceColor();
            // If we want to use solid colors
        } else if (!animationMode) {
            controller.clearAnimation(0);
            // If we have a valid animation and want to use it
        } else {
            controller.animate(currentAnimation, 0);
        }
    }
}
