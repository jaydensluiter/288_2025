package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

// import edu.wpi.first.math.controller.PIDController;

public class CoralPivot extends SubsystemBase {
    private final TalonFX pivotMotor;
    // private final PIDController pivotPID;

    /* Subsystem init */
    public CoralPivot() {
        // pivotPID.setTolerance(.01); // Set the tolerance for the PID controller
        pivotMotor = new TalonFX(Constants.CoralPivot.PIVOT_PORT);
        // pivotPID = new PIDController(
        //     Constants.CoralPivot.kP,
        //     Constants.CoralPivot.kI,
        //     Constants.CoralPivot.kD
        // );

    }

    public double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    /* Set point commands */

    // public Command setPosition(double Position){
    //     return run(
    //         () -> {
    //             PivotMotor.set(MathUtil.clamp(pivotPID.calculate(PivotMotor.getPosition().getValueAsDouble(), Position), -.9, .9)); // Set the pivot motor to the calculated PID value
    //         }).until(() -> pivotPID.atSetpoint());
    // }
}