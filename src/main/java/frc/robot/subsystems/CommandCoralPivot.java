package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.controller.PIDController;

public class CommandCoralPivot implements Subsystem{
    private final TalonFX PivotMotor = new TalonFX(13); // Coral arm pivot motor
    private final Encoder PivotEncoder = new Encoder(2, 3); // Encoder for the elevator lift

    private final PIDController pivotPID = new PIDController(0.1, 0, 0.1); // PID controller for the pivot motor

    private double kPrimeT4 = 0; // Setpoint for the pivot motor
    private double kStow = 0; // Setpoint for the pivot motor

    /* Subsystem init */
    public CommandCoralPivot() {
        pivotPID.setTolerance(1); // Set the tolerance for the PID controller
    }
    
    /* Set point commands */

    public Command setT4Command(){
        return run(
            () -> {
                PivotMotor.set(pivotPID.calculate(PivotMotor.getPosition().getValueAsDouble(), kPrimeT4)); // Set the pivot motor to the calculated PID value
            });
    }
}

