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

public class CommandHang implements Subsystem {
    private final TalonFX HangMotor = new TalonFX(18);
    

    public CommandHang() {
        
    }
    
    public Command Up(){
        return run(
            () -> {
                HangMotor.set(.9);
            });
    }

    public Command Down(){
        return run(
            () -> {
                HangMotor.set(-.9);
            });
    }

    public Command Stop(){
        return run(
            () -> {
                HangMotor.stopMotor();
            });
    }
}
