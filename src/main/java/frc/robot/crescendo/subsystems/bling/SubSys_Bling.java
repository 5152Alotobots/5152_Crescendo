package frc.robot.crescendo.subsystems.bling;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CAN_IDs.CANDLE_CAN_ID;
import static frc.robot.crescendo.subsystems.bling.SubSys_Bling_Constants.Animations.defaultAnimation;
import static frc.robot.crescendo.subsystems.bling.SubSys_Bling_Constants.LED_TYPE;
import static frc.robot.crescendo.subsystems.bling.SubSys_Bling_Constants.MAX_LED_BRIGHTNESS;

public class SubSys_Bling extends SubsystemBase {
    private final CANdle controller = new CANdle(CANDLE_CAN_ID);

    public SubSys_Bling() {
        controller.configBrightnessScalar(MAX_LED_BRIGHTNESS);
        controller.configLEDType(LED_TYPE);
        controller.animate(defaultAnimation);
        /*
        // Alliance colors
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
                controller.setLEDs(0, 0, 255);
            }
        } else {
            controller.setLEDs(255, 0, 0);
        }
        */

    }

    @Override
    public void periodic() {
    }
}
