package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Hang extends SubsystemBase {
    private final TalonFX hangMotor;

    public Hang() {
        hangMotor = new TalonFX(Constants.Hang.HANG_PORT);
    }
    
    public void moveUp(){
        hangMotor.set(Constants.Hang.HANG_SPEED);
    }

    public void moveDown(){
        hangMotor.set(-Constants.Hang.HANG_SPEED);
    }
}