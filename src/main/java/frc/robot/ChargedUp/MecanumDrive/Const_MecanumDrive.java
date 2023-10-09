package frc.robot.ChargedUp.MecanumDrive;

public final class Const_MecanumDrive {
  // Constants for @params
  public static final int k_FrontLeftMotorPort = 3;
  public static final int k_RearLeftMotorPort = 4;
  public static final int k_FrontRightMotorPort = 1;
  public static final int k_RearRightMotorPort = 2;
  public static final int k_TestMotor = 6;

  // TODO: TUNE PID
  /** Drive PID LINEAR */
  public static final double k_driveLinKp = 0.1;

  public static final double k_driveLinKi = 0;
  public static final double k_driveLinKd = 0;
  public static final double k_driveAngKp = 0.1;
  public static final double k_driveAngKi = 0;
  public static final double k_driveAngKd = 0;

  /** Encoders */
  public static final int[] k_FrontLeftEncoderPorts = new int[] {0, 1};

  public static final int[] k_RearLeftEncoderPorts = new int[] {2, 3};
  public static final int[] k_FrontRightEncoderPorts = new int[] {4, 5};
  public static final int[] k_RearRightEncoderPorts = new int[] {6, 7};

  public static final boolean k_FrontLeftEncoderReversed = false;
  public static final boolean k_RearLeftEncoderReversed = true;
  public static final boolean k_FrontRightEncoderReversed = false;
  public static final boolean k_RearRightEncoderReversed = true;

  public static final double k_TrackWidth = 0.5;
  // Distance between centers of right and left wheels on robot
  public static final double k_WheelBase = 0.7;
  // Distance between centers of front and back wheels on robot

}
