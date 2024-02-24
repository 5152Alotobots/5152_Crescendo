package frc.robot.crescendo.subsystems.shooter.util;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class DirectionUtils {

    /**
     * Converts the state of input and output joystick buttons to an intake direction.
     *
     * @param inButton  The button representing intake in.
     * @param outButton The button representing intake out.
     * @return The intake direction based on the state of the input and output buttons.
     */
    public static IntakeDirection toIntakeDirection(JoystickButton inButton, JoystickButton outButton) {
        if (inButton.getAsBoolean() && !outButton.getAsBoolean()) {
            return IntakeDirection.TRANSFER;
        } else if (!inButton.getAsBoolean() && outButton.getAsBoolean()) {
            return IntakeDirection.OUT;
        } else {
            return IntakeDirection.OFF;
        }
    }

    /**
     * Converts the state of a shoot joystick button to a shooter direction.
     *
     * @param shootButton The button representing shooting.
     * @return The shooter direction based on the state of the shoot button. (OUT/OFF)
     */
    public static ShooterDirection toShooterDirection(JoystickButton shootButton) {
        return shootButton.getAsBoolean() ? ShooterDirection.OUT : ShooterDirection.OFF;
    }
}