package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;

public class Elevator extends SubsystemBase {
    private final SparkMax elevatorMotor;
    private final Encoder elevatorEncoder;
    private final PIDController elevatorPID;

    public Elevator() {
        elevatorMotor = new SparkMax(Constants.Elevator.ELEVATOR_PORT, SparkMax.MotorType.kBrushless);
        elevatorEncoder = new Encoder(
            Constants.Elevator.ELEVATOR_PORT_ENCODER_CHANNEL_A, 
            Constants.Elevator.ELEVATOR_PORT_ENCODER_CHANNEL_B, 
            true,
            Encoder.EncodingType.k2X
        );
        elevatorPID = new PIDController(
            Constants.Elevator.kP,
            Constants.Elevator.kI,
            Constants.Elevator.kD
        );

        resetEncoder();
    }

    public void resetEncoder() {
        elevatorEncoder.reset();
    }

    public double getPosition() {
        return elevatorEncoder.getDistance();
    }

    public BooleanSupplier isAtPosition() {
        double setPoint = elevatorPID.getSetpoint();
        return () -> Math.abs(getPosition() - setPoint) < Constants.Elevator.kTolerance;
    }

    public void moveToSetPosition(double position) {
        elevatorPID.setSetpoint(position);
        elevatorMotor.set(elevatorPID.calculate(getPosition(), position));
    }

    public Command moveToSetPositionCommand(double position) {
        return run(() -> moveToSetPosition(position)).until(isAtPosition());
    }
}