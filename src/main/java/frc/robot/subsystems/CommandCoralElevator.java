package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CommandCoralElevator implements Subsystem {


    /* Motor objects */
    
    private final TalonFX LiftMotor = new TalonFX(14); // Elevator lift motor
    private final Encoder LiftEncoder = new Encoder(0, 1); // Encoder for the elevator lift
    

    /* Contants for control */
    private static final double ENCODER_TICKS_PER_DEGREE = 2048.0 / 360.0; // Translate ticks to degrees
    private static final double LIFT_MAX_SPEED = 0.75; // Max speed 
    private static final double SPOOL_RADIUS = 2; // Radius of the spool in inches

    /* Subsystem init */
    public CommandCoralElevator() {
        LiftEncoder.reset();
    }

    /**
     * Moves the arm to a specific angle.
     * @param targetAngle Target angle in degrees.
     */
    public void moveToAngle(double targetAngle) {
        double currentHight = getElevatorHeight();
        double error = targetAngle - currentHight;

        // Simple proportional control (P-Control)
        double kP = 0.02; // Adjust this constant for your system
        double speed = kP * error;

        // Limit the speed to avoid overshooting
        speed = Math.max(-LIFT_MAX_SPEED, Math.min(speed, LIFT_MAX_SPEED));

        // Run motor until close enough to the target angle
        while (Math.abs(error) > 1.0) { // 1-degree tolerance
            LiftMotor.set(speed);
            currentHight = getElevatorHeight();
            error = targetAngle - currentHight;
            speed = kP * error;
            speed = Math.max(-LIFT_MAX_SPEED, Math.min(speed, LIFT_MAX_SPEED));
        }

        LiftMotor.stopMotor();
    }

    /* Get arm angle */
    public double getElevatorHeight() {
        double degreesTurned = LiftEncoder.getDistance() / ENCODER_TICKS_PER_DEGREE;
        double circumference = 2 * Math.PI * SPOOL_RADIUS;
        double height = (degreesTurned / 360.0) * circumference;
        return height;
    }

}
