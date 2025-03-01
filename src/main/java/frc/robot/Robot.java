// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.CommandCoralPivot;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public TalonFX pivot = new TalonFX(13);
  public SparkMax elevator = new SparkMax(14, MotorType.kBrushless);
  public TalonFX hang = new TalonFX (18);
  public Encoder elevatorEncoder = new Encoder(0, 1);
  public SparkMax algaepivot = new SparkMax(19, MotorType.kBrushless);
  public TalonFX algaemech = new TalonFX(17);

  private final CommandPS4Controller Driver = new CommandPS4Controller(0);
  private final CommandXboxController operator = new CommandXboxController(1);



  // Define a gravity constant
  private static final double kG = .05; // Adjust this value as needed

  // Define constants for the P loop
  private static final double kP = 1; // Proportional gain, adjust as needed
  private double targetAngle = 0.0; // Target angle for the pivot
  private boolean targetAngleSet = false; // Flag to track if the target angle has been set

  // Define constants
  private static final double SPOOL_DIAMETER_INCHES = 1.5; // Diameter of the spool in inches
  private static final double kP_ELEVATOR = 1; // Proportional gain for the elevator control loop
  private double targetHeightInches = 0.0; // Target height for the elevator
  private boolean targetHeightSet = false; // Flag to track if the target height has been set

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  private void holdElevatorInPlace() {
    elevator.set(kG); // Apply a constant force to counteract gravity
  }

  private void movePivotToAngle() {
    double currentAngle = pivot.getPosition().getValueAsDouble(); // Get the current angle of the pivot
    double error = targetAngle - currentAngle; // Calculate the error
    double output = kP * error; // Calculate the output using the P loop
    pivot.set(output); // Set the pivot motor to the calculated output
  }

  // Method to set the target angle for the pivot
  public void setPivotTargetAngle(double angle) {
    targetAngle = angle;
  }

  // Method to move the elevator to a set height
  public void moveElevatorToHeight(double targetHeightInches) {
    double currentHeightInches = getElevatorHeightInches(); // Get the current height of the elevator
    double error = targetHeightInches - currentHeightInches; // Calculate the error
    double output = kP_ELEVATOR * error; // Calculate the output using the P loop
    elevator.set(output); // Set the elevator motor to the calculated output
  }

  // Method to get the current height of the elevator based on the encoder value
  private double getElevatorHeightInches() {
    double encoderValue = elevatorEncoder.getDistance(); // Get the encoder value
    double spoolCircumferenceInches = Math.PI * SPOOL_DIAMETER_INCHES; // Calculate the circumference of the spool in inches
    return encoderValue * spoolCircumferenceInches; // Calculate the height based on the encoder value
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    elevator.stopMotor();
    pivot.setPosition(0);
    
  }

  @Override
  public void teleopPeriodic() {
    // // Call the P loop function to move the pivot to the target angle
    // movePivotToAngle();

    // // Example condition to set the target angle once
    // if (operator.povDown().getAsBoolean() && !targetAngleSet) {
    //     setPivotTargetAngle(0.0); // Set the target angle to 0 degrees
    //     targetAngleSet = true; // Mark the target angle as set
    // }
    // if (operator.povUp().getAsBoolean() && !targetAngleSet) {
    //     setPivotTargetAngle(185.0); // Set the target angle to 180 degrees
    //     targetAngleSet = true; // Mark the target angle as set
    // }
    // if (operator.povRight().getAsBoolean() && !targetAngleSet) {
    //     setPivotTargetAngle(100.0); // Set the target angle to 90 degrees
    //     targetAngleSet = true; // Mark the target angle as set
      
    // } else {
      
    // }

    // // // Example condition to set the target height once
    // // if (operator.a().getAsBoolean() && !targetHeightSet) {
    // //     moveElevatorToHeight(24.0); // Set the target height to 24 inches
    // //   targetHeightSet = true; // Mark the target height as set
    // // }
    // // if (operator.b().getAsBoolean() && !targetHeightSet) {
    // //     moveElevatorToHeight(48.0); // Set the target height to 48 inches
    // //     targetHeightSet = true; // Mark the target height as set
    // // }
    // // if (operator.x().getAsBoolean() && !targetHeightSet) {
    // //     moveElevatorToHeight(72.0); // Set the target height to 72 inches
    // //     targetHeightSet = true; // Mark the target height as set
    // // }

    if (operator.a().getAsBoolean()) {
      elevator.set(-0.70); // Move the elevator up at 50% power
    } else if (operator.y().getAsBoolean()) {
      elevator.set(0.70); // Move the elevator down at 50% power
    } else {
      holdElevatorInPlace(); // Stop the elevator motor
    }

    // if (operator.povUp().getAsBoolean()) {
    //   pivot.set(.3); // Move the pivot up at 50% power
    // } else if (operator.povDown().getAsBoolean()) {
    //   pivot.set(-0.3); // Move the pivot down at 50% power
    // } else {
    //   pivot.stopMotor(); // Stop the pivot motor
    // }

    // if (operator.x().getAsBoolean()) {
    //   hang.set(-0.9); // Move the hang up at 50% power
    // } else if (operator.b().getAsBoolean()) {
    //   hang.set(0.9); // Move the hang down at 50% power
    // } else {
    //   hang.stopMotor(); // Stop the hang motor
    // }

    // if (operator.povRight().getAsBoolean()) {
    //   algaepivot.set(-0.9); // Move the algaepivot up at 50% power
    // } else if (operator.povLeft().getAsBoolean()) {
    //   algaepivot.set(0.9); // Move the algaemech down at 50% power
    // } else {
    //   algaepivot.stopMotor(); // Stop the algaemech motor
    // }

    // if (operator.rightTrigger().getAsBoolean()) {
    //   algaemech.set(-0.9); // Move the algaemech up at 50% power
    // } else if (operator.leftTrigger().getAsBoolean()) {
    //   algaemech.set(0.9); // Move the algaemech down at 50% power
    // } else {
    //   algaemech.stopMotor(); // Stop the algaemech motor
    // }

    SmartDashboard.putString("Pivot Position", String.valueOf(pivot.getPosition().getValueAsDouble()));
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}