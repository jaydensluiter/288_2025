package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Algae extends SubsystemBase{
     private final SparkMax algaeWrist;
     private final TalonFX algaeIntake;

    public Algae() {
        algaeWrist = new SparkMax(Constants.Algae.WRIST_PORT, SparkMax.MotorType.kBrushless);
        algaeIntake = new TalonFX(Constants.Algae.INTAKE_PORT);
    }

    public void moveWristUp(){
        algaeWrist.set(Constants.Algae.WRIST_SPEED);
    }

    public void moveWristDown(){
        algaeWrist.set(-Constants.Algae.WRIST_SPEED);
    }

    public void intake() {
        algaeIntake.set(Constants.Algae.INTAKE_SPEED);
    }

    public void outtake() {
        algaeIntake.set(Constants.Algae.OUTTAKE_SPEED);
    }
}
