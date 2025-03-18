package frc.robot;

public final class Constants {

    public static final class Elevator {
        public static final int ELEVATOR_PORT = 14;
        public static final int ELEVATOR_PORT_ENCODER_CHANNEL_A = 0;
        public static final int ELEVATOR_PORT_ENCODER_CHANNEL_B = 1;  

        public static final double ELEVATOR_GEAR_RATIO = 0;

        public static final double kP = 8;
        public static final double kI = 1.5;
        public static final double kD = 4;

        public static final double kTolerance = .1;

        public static final double kStow = 0;
        public static final double kLoad = 12000;
        public static final double kStab = 500;
        public static final double kL2 = 11000;
        public static final double kL3 = 10000;
        public static final double kL4 = 21500;
    }

    public static final class CoralPivot {
        public static final int PIVOT_PORT = 13;

        public static final double PIVOT_GEAR_RATIO = 0;

        public static final double kP = 8;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kTolerance = .1;

        public static final double kL4 = 0;
        public static final double kL3 = 0;
        public static final double kL2 = 0;
        public static final double kLoad = 0;
        public static final double kStow = 0;
    }

    public static final class Hang {
        public static final int HANG_PORT = 18;

        public static final double HANG_SPEED = .9;
    }

    public static final class Algae {
        public static final int WRIST_PORT = 19;
        public static final int INTAKE_PORT = 17;

        public static final double WRIST_SPEED = .25;
        public static final double INTAKE_SPEED = .35;
        public static final double OUTTAKE_SPEED = .8;
    }
}
