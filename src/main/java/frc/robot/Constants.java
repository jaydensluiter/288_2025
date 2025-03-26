package frc.robot;

public final class Constants {
    public static final class Elevator {
        public static final int ELEVATOR_PORT = 14;
        public static final int ELEVATOR_PORT_ENCODER_CHANNEL_A = 0;
        public static final int ELEVATOR_PORT_ENCODER_CHANNEL_B = 1;  


        public static final double kP = 8;
        public static final double kI = 1.5;
        public static final double kD = 4;
        public static final double kG = .025;

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

        public static final double PIVOT_SPEED = .45;
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

    public static final class Swerve {
        public static final double MAX_SPEED = 3.7;

        public static final double kSteerP = 100;
        public static final double kSteerI = 0;
        public static final double kSteerD = 0.5;
        public static final double kSteerS = 0.1;
        public static final double kSteerV = 2.33;
        public static final double kSteerA = 0.0;

        public static final double kDriveP = 0.1;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kDriveS = 0;
        public static final double kDriveV = 0.0;
        public static final double kDriveA = 0.124;
    }

    public static final class Controllers {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }
}
