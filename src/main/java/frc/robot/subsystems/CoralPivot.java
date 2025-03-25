package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

// import edu.wpi.first.math.controller.PIDController;

public class CoralPivot extends SubsystemBase {
    private final TalonFX pivotMotor;

    /* Subsystem init */
    public CoralPivot() {
        pivotMotor = new TalonFX(Constants.CoralPivot.PIVOT_PORT);

    }

    public double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public void moveUp() {
        pivotMotor.set(-Constants.CoralPivot.PIVOT_SPEED);
    }

    public void moveDown() {
        pivotMotor.set(Constants.CoralPivot.PIVOT_SPEED);
    }

    public void setDefault() {
        pivotMotor.set(0);
    }

    public void set(double speed) {
        pivotMotor.set(speed);
    }
}