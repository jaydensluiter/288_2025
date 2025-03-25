package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
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
        elevatorPID.setTolerance(Constants.Elevator.kTolerance);
        resetEncoder();
    }

    public void resetEncoder() {
        elevatorEncoder.reset();
    }

    public double getPosition() {
        return elevatorEncoder.getDistance();
    }

    public void setGravity() {
        elevatorMotor.set(Constants.Elevator.kG);
    }

    public void manualMoveUp() {
        moveToSetPositionCommand(getPosition() + .25);
    }

    public void manualMoveDown() {
        moveToSetPositionCommand(getPosition() - .25);
    }

    public Command moveToSetPositionCommand(double position) {
        return run(() -> {
            elevatorMotor.set(MathUtil.clamp(elevatorPID.calculate(elevatorEncoder.getDistance(), position), -0.9, 0.9));
        }).until(() -> elevatorPID.atSetpoint());
    }
}