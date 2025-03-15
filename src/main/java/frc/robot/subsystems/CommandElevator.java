package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class CommandElevator implements Subsystem {
    private final SparkMax elevator = new SparkMax(14, MotorType.kBrushless);
    private final Encoder elevatorEncoder = new Encoder(0, 1, true, Encoder.EncodingType.k2X); // Encoder for the elevator
    private double manualSpeed = 0;
    private static final double deadzone = .1;


    private final PIDController elevatorPID = new PIDController(
        ElevatorConstants.kElevatorP, //kp
        ElevatorConstants.kElevatorI, //ki
        ElevatorConstants.kElevatorD //kd
    ); // PID controller for the elevator motor

    /* Subsystem init */
    public CommandElevator() {
        elevatorPID.setTolerance(ElevatorConstants.kElevatorTolerance); // Set the tolerance for the PID controller
        elevatorEncoder.reset();
        elevatorPID.reset();
    }

    public void resetElevatorEncoder() {
        elevatorEncoder.reset();
    }

    // // MANUAL CONTROL ELEVATOR
    // public Command manualControl(CommandPS4Controller controller) {
    //     return run(() -> {
    //         double joystickValue = -controller.getRawAxis(1);
    //         if (Math.abs(joystickValue) < deadzone) joystickValue = 0;
    //         manualSpeed = joystickValue;
    //     });
    // }

    
    public boolean isSafe() { //detect weather or not the elevator is above the safe position
        boolean isAboveSafePosition = elevatorEncoder.getDistance() > ElevatorConstants.kSafePosition;
        SmartDashboard.putBoolean("Elevator Above Safe Position", isAboveSafePosition);
        return isAboveSafePosition;
    }
    
    public Double getElevatorPosition() {
        return elevatorEncoder.getDistance();
    }

    public Command setDefault() {
        return run(() -> {
            elevator.set(.025);
            // MANUAL CONTROL ELEVATOR
            // elevator.manualControl(controller);
        });
    }

    /* Set point commands */

    public Command setPosition(double position) {
        return run(() -> {
            elevator.set(MathUtil.clamp(elevatorPID.calculate(elevatorEncoder.getDistance(), position), -0.9, 0.9));
        }).until(() -> elevatorPID.atSetpoint());
    }
}
