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
    private double kPrimet3 = .15; // Setpoint for the pivot motor
    private double kPrimeT2 = .15; // Setpoint for the pivot motor
    private double kPrimeLoad = .51; // Setpoint for the pivot motor
    private double kStow = 0; // Setpoint for the pivot motor

    /* Subsystem init */
    public CommandCoralPivot() {
        pivotPID.setTolerance(.01); // Set the tolerance for the PID controller
        PivotMotor.setPosition(0);
        pivotPID.reset();
    }
    
    /* Set point commands */

    public Command setT4Command(){
        return run(
            () -> {
                PivotMotor.set(MathUtil.clamp(pivotPID.calculate(PivotMotor.getPosition().getValueAsDouble(), kPrimeT4), -.9, .9)); // Set the pivot motor to the calculated PID value
            });
    }

    public Command setT3Command(){
        return run(
            () -> {
                PivotMotor.set(MathUtil.clamp(pivotPID.calculate(PivotMotor.getPosition().getValueAsDouble(), kPrimet3), -.9, .9)); // Set the pivot motor to the calculated PID value
            });
    }

    public Command setLoadCommand(){
        return run(
            () -> {
                PivotMotor.set(MathUtil.clamp(pivotPID.calculate(PivotMotor.getPosition().getValueAsDouble(), kPrimeLoad), -.9, .9)); // Set the pivot motor to the calculated PID value
            });
    }
}