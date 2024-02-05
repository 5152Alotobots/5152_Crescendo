package frc.robot.library.drivetrains.mecanum;

public class SubSys_MecanumDrive_Constants {
    private SubSys_MecanumDrive_Constants() {}
    public static class AxisInverts {
        private AxisInverts () {}
        public static final boolean INVERT_FWD_AXIS = true;
        public static final boolean INVERT_STR_AXIS = true;
        public static final boolean INVERT_ROT_AXIS = true;
    }

    public static class MotorInverts {
        private MotorInverts () {}
        public static final boolean INVERT_FRONTLEFT = true;
        public static final boolean INVERT_FRONTRIGHT = true;
        public static final boolean INVERT_BACKLEFT = true;
        public static final boolean INVERT_BACKRIGHT = false;
    }
    public static final double DEADBAND = 0.5;
}
