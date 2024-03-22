package frc.robot.crescendo.subsystems.intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SubSys_Intake_Constants {

    // ********************
    // Intake Roller
    // ********************
    
    // Intake Roller - Motor
    // Neo 550 Spark Max
    public static final class IntakeRollerMtr{
      
        public static final double OpenLoopRampRate = 0.5; // seconds
        public static final class PID {
            public static final double Pgain = 0.0;
            public static final double Igain = 0.0;
            public static final double Dgain = 0.0;
            public static final double Izone = 0.0;
            public static final double FFwd = 0.0;
            public static final double MaxOutput = 1;
            public static final double MinOutput = -1;
            public static final double MaxRPM = 5700;
            public static final double ClosedLoopRampRate = 1; // seconds
        }
        public static final class SpeedSettings{
            // Duty Cycle
            public static final double MaxDutyCycle = 1.0;  // 0-1
            public static final double IntakeDutyCycle = -0.75;
            public static final double OuttakeDutyCycle = 0.25;
            public static final double TransferDutyCycle = 0.25;
        }
    
    }
    // Intake Roller - Encoder
    // Intake Roller IR Sensor

    // ********************
    // Intake Arm
    // ********************
    public static final class IntakeArm{
  
        public static final class IntakeArmPositions{
            public static final double IntakeArmMaxPos = -0.015;           // Ground Position
            public static final double IntakeArmMinPos = -0.44775;          // Up Position
            public static final double IntakeArmPickupPos = -0.015;        // Note Pickup Position
            public static final double IntakeArmTransferPos = -0.44775;    // Transfer Position
        }
        public static final class SpeedSettings{
            // Voltage
            // Velocity
            public static final double IntakeArmMaxVelocity = 0.5;  // Rotations/s
            // Acceleration
            public static final double IntakeArmMaxAcceleration = 0.75; // Rotations/s/s
            // Jerk 
            public static final double IntakeArmMaxJerk = 0.0; // Rotations/s/s/s
        }
        public static final class ErrorTolerances{
            public static final double PositionErrorTolerance = 0.01;  //Rotations
            public static final double VelocityErrorTolerance = 0.075;  //Rotations/second
        }
        // Intake Arm Motor
        public static final class IntakeArmMtr{

            private static final FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                // .withFeedbackRemoteSensorID(intakeArmCANCoder.getDeviceID()  Set in SubSystem
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                .withRotorToSensorRatio(1)
                .withSensorToMechanismRatio(0);   
            
            private static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive)
                .withPeakForwardDutyCycle(0.45)     
                .withPeakReverseDutyCycle(-0.45)
                .withDutyCycleNeutralDeadband(0.05);
                
            private static final OpenLoopRampsConfigs openLoopRampConfigs = new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(0.75)
                .withVoltageOpenLoopRampPeriod(.75)
                .withTorqueOpenLoopRampPeriod(.75);
            
            private static final SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)    
                .withForwardSoftLimitThreshold(IntakeArmPositions.IntakeArmMaxPos)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(IntakeArmPositions.IntakeArmMinPos);

            private static final HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs()
                .withForwardLimitEnable(true)
                .withForwardLimitSource(ForwardLimitSourceValue.LimitSwitchPin)
                .withReverseLimitEnable(true)
                .withForwardLimitSource(ForwardLimitSourceValue.LimitSwitchPin);

            private static final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(false)
                .withStatorCurrentLimit(200)
                .withSupplyCurrentLimitEnable(false)
                .withSupplyCurrentLimit(200)
                .withSupplyTimeThreshold(0.5)
                .withSupplyCurrentThreshold(220);

            // Velocity Control
            public static final Slot0Configs slot0Configs = new Slot0Configs()
                // FeedForward
                .withKS(0.0)  // Add -0.075 V output to overcome static friction
                .withKV(25.0)   // A velocity target of 1 rps results in -0.05 V output
                .withKA(0) // An acceleration of 1 rps/s requires 0.01 V output
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKG(-0.3)   // Voltage to hold arm horizontal
                // FeedBack
                .withKP(1)
                .withKI(0)
                .withKD(0);

            // Position Control
            public static final Slot1Configs slot1Configs = new Slot1Configs()
                // FeedForward
                .withKS(0.0)  // Add -0.075 V output to overcome static friction
                .withKV(25.0)   // A velocity target of 1 rps results in -0.05 V output
                .withKA(0) // An acceleration of 1 rps/s requires 0.01 V output
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKG(-0.3)   // Voltage to hold arm horizontal
                // FeedBack
                .withKP(50)
                .withKI(0)
                .withKD(0);

            private static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(SpeedSettings.IntakeArmMaxVelocity)
                .withMotionMagicAcceleration(SpeedSettings.IntakeArmMaxAcceleration)
                .withMotionMagicJerk(SpeedSettings.IntakeArmMaxJerk);

            private static final ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs()
                .withVoltageClosedLoopRampPeriod(0.25);
                
            public static final TalonFXConfiguration intakeArmMtrConfiguration = new TalonFXConfiguration()
                .withFeedback(feedbackConfigs)
                .withMotorOutput(motorOutputConfigs)
                .withOpenLoopRamps(openLoopRampConfigs)
                .withSoftwareLimitSwitch(softwareLimitSwitchConfigs)
                .withHardwareLimitSwitch(hardwareLimitSwitchConfigs)
                .withCurrentLimits(currentLimitsConfigs)
                .withSlot0(slot0Configs)
                .withSlot1(slot1Configs)
                .withMotionMagic(motionMagicConfigs)
                .withClosedLoopRamps(closedLoopRampsConfigs);
        }

        // Intake Arm CANcoder
        public static final class IntakeArmCANCoder {
            
            private static final MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs()
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                .withMagnetOffset(0.167);

            public static final CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration()
                .withMagnetSensor(magnetSensorConfigs);
        }
    }
}
