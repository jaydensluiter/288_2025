package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.controller.PIDController;

public class CommandCoralPivot implements Subsystem{
    private final TalonFX PivotMotor = new TalonFX(13); // Coral arm pivot motor

    public PIDController pivotPID = new PIDController(8, 0, 0.1); // PID controller for the pivot motor

    private double kPrimeT4 = 0; // Setpoint for the pivot motor
    private double kPrimeT3 = .15; // Setpoint for the pivot motor
    private double kPrimeT2 = .15; // Setpoint for the pivot motor
    private double kPrimeLoad = .51; // Setpoint for the pivot motor
    private double kStow = 0; // Setpoint for the pivot motor

    /* Subsystem init */
    public CommandCoralPivot() {
        pivotPID.setTolerance(.01); // Set the tolerance for the PID controller
        PivotMotor.setPosition(0);
        pivotPID.reset();
    }

    public Command setBrake(){
        return run(
            () -> {
                PivotMotor.set(0); // Set the pivot motor to the calculated PID value
            });
    }

    /* Set point commands */

    public Command setStowCommand(){
        return run(
            () -> {
                PivotMotor.set(MathUtil.clamp(pivotPID.calculate(PivotMotor.getPosition().getValueAsDouble(), kStow), -.9, .9)); // Set the pivot motor to the calculated PID value
            }).until(() -> pivotPID.atSetpoint());
    }

    public Command setLoadCommand(){
        return run(
            () -> {
                PivotMotor.set(MathUtil.clamp(pivotPID.calculate(PivotMotor.getPosition().getValueAsDouble(), kPrimeLoad), -.5, .5)); // Set the pivot motor to the calculated PID value
            }).until(() -> pivotPID.atSetpoint());
    }

    public Command setT4Command(){
        return run(
            () -> {
                PivotMotor.set(MathUtil.clamp(pivotPID.calculate(PivotMotor.getPosition().getValueAsDouble(), kPrimeT4), -.9, .9)); // Set the pivot motor to the calculated PID value
            }).until(() -> pivotPID.atSetpoint());
    }

    public Command setT3Command(){
        return run(
            () -> {
                PivotMotor.set(MathUtil.clamp(pivotPID.calculate(PivotMotor.getPosition().getValueAsDouble(), kPrimeT3), -.9, .9)); // Set the pivot motor to the calculated PID value
            }).until(() -> pivotPID.atSetpoint());
    }

    public Command setT2Command(){
        return run(
            () -> {
                PivotMotor.set(MathUtil.clamp(pivotPID.calculate(PivotMotor.getPosition().getValueAsDouble(), kPrimeT2), -.9, .9)); // Set the pivot motor to the calculated PID value
            }).until(() -> pivotPID.atSetpoint());
    }

    public Command setPosition(double Position){
        return run(
            () -> {
                PivotMotor.set(MathUtil.clamp(pivotPID.calculate(PivotMotor.getPosition().getValueAsDouble(), Position), -.9, .9)); // Set the pivot motor to the calculated PID value
            }).until(() -> pivotPID.atSetpoint());
    }
}