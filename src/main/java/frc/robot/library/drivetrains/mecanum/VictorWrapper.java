package frc.robot.library.drivetrains.mecanum;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class VictorWrapper extends VictorSPX {
    public VictorWrapper(int deviceNumber) {
		super(deviceNumber);
    }

    public void set(double percentOutput) {
        super.set(VictorSPXControlMode.PercentOutput, percentOutput);
    }
}
