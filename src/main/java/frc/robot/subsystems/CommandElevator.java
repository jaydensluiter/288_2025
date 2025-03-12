package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CommandElevator implements Subsystem {
    private final SparkMax elevator = new SparkMax(14, MotorType.kBrushless);
    private final Encoder elevatorEncoder = new Encoder(0, 1, true, Encoder.EncodingType.k2X); // Encoder for the elevator

    private final PIDController elevatorPID = new PIDController(
        400, //kp
        0, //ki
        175 //kd
    ); // PID controller for the elevator motor

    private static final double SPOOL_DIAMETER_INCHES = 1.5;
    private static final double SPOOL_CIRCUMFERENCE = Math.PI * SPOOL_DIAMETER_INCHES;
    private static final double INCHES_PER_ROTATION = SPOOL_CIRCUMFERENCE;

    /* Subsystem init */
    public CommandElevator() {
        elevatorPID.setTolerance(ElevatorConstants.kElevatorTolerance); // Set the tolerance for the PID controller
        elevatorEncoder.reset();
        elevatorPID.reset();
    }

    public boolean isSafe() { //detect weather or not the elevator is above the safe position
        boolean isAboveSafePosition = elevatorEncoder.getDistance() > ElevatorConstants.kSafePosition;
        SmartDashboard.putBoolean("Elevator Above Safe Position", isAboveSafePosition);
        return isAboveSafePosition;
    }
    
    public Double getElevatorPosition() {
        return elevatorEncoder.getDistance();
    }

    public Command setGravity() {
        return run(() -> {
            elevator.set(.025);
        });
    }

    /* Set point commands */
    public Command setStowPosition() {
        return run(() -> {
            elevator.set(MathUtil.clamp(elevatorPID.calculate(elevatorEncoder.getDistance(), ElevatorConstants.kStowPosition), -0.9, 0.9));
        }).until(() -> elevatorPID.atSetpoint());
    }

    public Command setLoadingPosition() {
        return run(() -> {
            elevator.set(MathUtil.clamp(elevatorPID.calculate(elevatorEncoder.getDistance(), ElevatorConstants.kLoadingPosition), -0.9, 0.9));
            SmartDashboard.putString("Elevator Position", String.valueOf(elevatorEncoder.getDistance()));
        }).until(() -> elevatorPID.atSetpoint());
    }

    public Command setStabPosition() {
        return run(() -> {
            elevator.set(MathUtil.clamp(elevatorPID.calculate(elevatorEncoder.getDistance(), ElevatorConstants.kStabPosition), -0.9, 0.9));
            SmartDashboard.putString("Elevator Position", String.valueOf(elevatorEncoder.getDistance()));
        }).until(() -> elevatorPID.atSetpoint());
    }

    public Command setT2Position() {
        return run(() -> {
            elevator.set(MathUtil.clamp(elevatorPID.calculate(elevatorEncoder.getDistance(), ElevatorConstants.kT2Position), -0.9, 0.9));
            SmartDashboard.putString("Elevator Position", String.valueOf(elevatorEncoder.getDistance()));
        }).until(() -> elevatorPID.atSetpoint());
    }

    public Command setT3Position() {
        return run(() -> {
            elevator.set(MathUtil.clamp(elevatorPID.calculate(elevatorEncoder.getDistance(), ElevatorConstants.kT3Position), -0.9, 0.9));
        }).until(() -> elevatorPID.atSetpoint());
    }

    public Command setT4Position() {
        return run(() -> {
            elevator.set(MathUtil.clamp(elevatorPID.calculate(elevatorEncoder.getDistance(), ElevatorConstants.kT4Position), -0.9, 0.9));
        }).until(() -> elevatorPID.atSetpoint());
    }

    public Command setT4ScorePosition() {
        return run(() -> {
            elevator.set(MathUtil.clamp(elevatorPID.calculate(elevatorEncoder.getDistance(), ElevatorConstants.kT3Position), -0.9, 0.9));
        }).until(() -> elevatorPID.atSetpoint());
    }

    public Command setPosition(double position) {
        return run(() -> {
            elevator.set(MathUtil.clamp(elevatorPID.calculate(elevatorEncoder.getDistance(), position), -0.9, 0.9));
        }).until(() -> elevatorPID.atSetpoint());
    }
}
