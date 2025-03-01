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

public class CommandElevator implements Subsystem{
    public SparkMax elevator = new SparkMax(14, MotorType.kBrushless);
    private final Encoder elevatorEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k2X); // Encoder for the elevator

    public PIDController elevatorPID = new PIDController(8, 0, 0.1); // PID controller for the pivot motor

    private double kPrimeT4 = 0; // Setpoint for the pivot motor
    private double kPrimeT3 = .15; // Setpoint for the pivot motor
    private double kPrimeT2 = .15; // Setpoint for the pivot motor
    private double kPrimeLoad = .51; // Setpoint for the pivot motor
    private double kStow = 0; // Setpoint for the pivot motor

    /* Subsystem init */
    public CommandElevator() {
        elevatorPID.setTolerance(.01); // Set the tolerance for the PID controller
        elevatorEncoder.reset();
        elevatorPID.reset();
    }
    
    /* Set point commands */

    public Command sethightCommand(){
        return run(
            () -> {
                elevator.set(MathUtil.clamp(elevatorPID.calculate(elevatorEncoder.getDistance(), kPrimeT4), -.9, .9)); // Set the pivot motor to the calculated PID value
            });
    }
}
