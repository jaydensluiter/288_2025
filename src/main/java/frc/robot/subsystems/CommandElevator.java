package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.controller.PIDController;

public class CommandElevator implements Subsystem {
    private final SparkMax elevator = new SparkMax(14, MotorType.kBrushless);
    private final Encoder elevatorEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k2X); // Encoder for the elevator

    private final PIDController elevatorPID = new PIDController(8, 0, 0.1); // PID controller for the elevator motor

    private static final double SPOOL_DIAMETER_INCHES = 1.5;
    private static final double SPOOL_CIRCUMFERENCE = Math.PI * SPOOL_DIAMETER_INCHES;
    private static final double INCHES_PER_ROTATION = SPOOL_CIRCUMFERENCE;

    // Setpoints for the elevator in rotations
    private static final double STOW_POSITION_ROTATIONS = 0;
    private static final double LOADING_POSITION_ROTATIONS = 8 / INCHES_PER_ROTATION;
    private static final double T2_POSITION_ROTATIONS = 10 / INCHES_PER_ROTATION;
    private static final double T3_POSITION_ROTATIONS = 15 / INCHES_PER_ROTATION;
    private static final double T4_POSITION_ROTATIONS = 20 / INCHES_PER_ROTATION;
    private static final double T4_SCORE_POSITION_ROTATIONS = 25 / INCHES_PER_ROTATION;

    /* Subsystem init */
    public CommandElevator() {
        elevatorPID.setTolerance(0.01); // Set the tolerance for the PID controller
        elevatorEncoder.reset();
        elevatorPID.reset();
    }

    /* Set point commands */
    public Command setStowPosition() {
        return run(() -> {
            elevator.set(MathUtil.clamp(elevatorPID.calculate(elevatorEncoder.getDistance(), STOW_POSITION_ROTATIONS), -0.9, 0.9));
        });
    }

    public Command setLoadingPosition() {
        return run(() -> {
            elevator.set(MathUtil.clamp(elevatorPID.calculate(elevatorEncoder.getDistance(), LOADING_POSITION_ROTATIONS), -0.9, 0.9));
        });
    }

    public Command setT2Position() {
        return run(() -> {
            elevator.set(MathUtil.clamp(elevatorPID.calculate(elevatorEncoder.getDistance(), T2_POSITION_ROTATIONS), -0.9, 0.9));
        });
    }

    public Command setT3Position() {
        return run(() -> {
            elevator.set(MathUtil.clamp(elevatorPID.calculate(elevatorEncoder.getDistance(), T3_POSITION_ROTATIONS), -0.9, 0.9));
        });
    }

    public Command setT4Position() {
        return run(() -> {
            elevator.set(MathUtil.clamp(elevatorPID.calculate(elevatorEncoder.getDistance(), T4_POSITION_ROTATIONS), -0.9, 0.9));
        });
    }

    public Command setT4ScorePosition() {
        return run(() -> {
            elevator.set(MathUtil.clamp(elevatorPID.calculate(elevatorEncoder.getDistance(), T4_SCORE_POSITION_ROTATIONS), -0.9, 0.9));
        });
    }
}
