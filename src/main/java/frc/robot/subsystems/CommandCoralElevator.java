package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;

public class CommandCoralElevator {

    /* PID controllers */
    private final PIDController m_liftController = new PIDController(7, 0, 0);
    private final PIDController m_PivotController = new PIDController(7, 0, 0);

    /* Motor objects */
    private final TalonFX PivotMotor = new TalonFX(13); // Coral arm pivot motor
    private final TalonFX LiftMotor = new TalonFX(14); // Elevator lift motor
    private final Encoder LiftEncoder = new Encoder(0, 1); // Encoder for the elevator lift

    /* Contants for control */
    private static final double ENCODER_TICKS_PER_DEGREE = 2048.0 / 360.0; // Translate ticks to degrees
    private static final double ARM_HOLD_SPEED = 0.1; // Speed to hold the arm
    private static final double ARM_MAX_SPEED = 0.75; // Max speed 

    /* Subsystem init */
    public CommandCoralElevator() {
        LiftEncoder.reset();
    }


    /**
     * Moves the arm to a specific angle.
     * @param targetAngle Target angle in degrees.
     */
    public void moveToAngle(double targetAngle) {
        double currentAngle = getArmAngle();
        double error = targetAngle - currentAngle;

        // Simple proportional control (P-Control)
        double kP = 0.02; // Adjust this constant for your system
        double speed = kP * error;

        // Limit the speed to avoid overshooting
        speed = Math.max(-ARM_MAX_SPEED, Math.min(speed, ARM_MAX_SPEED));

        // Run motor until close enough to the target angle
        while (Math.abs(error) > 1.0) { // 1-degree tolerance
            LiftMotor.set(speed);
            currentAngle = getArmAngle();
            error = targetAngle - currentAngle;
            speed = kP * error;
            speed = Math.max(-ARM_MAX_SPEED, Math.min(speed, ARM_MAX_SPEED));
        }

        LiftMotor.stopMotor();
    }

    /* Get arm angle */
    public double getArmAngle() {
        return LiftEncoder.getDistance() / ENCODER_TICKS_PER_DEGREE;
    }
}
