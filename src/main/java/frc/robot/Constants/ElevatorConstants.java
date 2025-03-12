package frc.robot.Constants;

public class ElevatorConstants {
    /* PID values */
    public static final double kElevatorP = 400;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 175;

    public static final double kElevatorTolerance = 75;

    public static final double kFeedForwardS = (0.95 - 0.2)/2*0.8;   /* kG too high - kG too low / 2  0.95, 0.2 */
    public static final double kFeedForwardG = (0.95 + 0.2)/2;  /* kG too high + kG too low / 2 */    // calculated value 0.6
    public static final double kFeedForwardV = 0.12;   // calculated value 0.12

    /* Positions */
    public static final double kStowPosition = 0;
    public static final double kSafePosition = 8000;
    public static final double kLoadingPosition = 10000;
    public static final double kStabPosition = 5000;
    public static final double kT2Position = 11000;
    public static final double kT3Position = 15000;
    public static final double kT4Position = 17000;
}
