// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CommandsManager;
import frc.robot.commands.CoralPivotCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.controls.DriverController;
import frc.robot.controls.OperatorController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Autons;

public class RobotContainer {

    // private final SendableChooser<Command> autoChooser;

    // Flag to track the current speed mode
    // private boolean isSlowSpeed = false;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double SlowSpeed = MaxSpeed * 0.2; // 20% of top speed
    // private double speedMultiplier = isSlowSpeed ? slowSpeed : 1.0;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Define the slow speed multiplier
    // private static double slowSpeed = 1; // Adjust this value as needed

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Define the button to toggle speed mode
    private final DriverController driverController;
    private final OperatorController operatorController;

    private final SwerveDrive swerveDrive;
    private final CoralPivot coralPivot;
    private final Elevator elevator;
    private final Hang hang;
    private final Algae algae;
    private final CommandsManager commandsManager;
    private final Autons autons;

    public RobotContainer() {
        commandsManager = new CommandsManager();

        driverController = new DriverController(Constants.Controllers.DRIVER_PORT);
        operatorController = new OperatorController(Constants.Controllers.OPERATOR_PORT);

        swerveDrive = TunerConstants.createDrivetrain();
        coralPivot = new CoralPivot();
        elevator = new Elevator();
        hang = new Hang();
        algae = new Algae();
        autons = new Autons();

        elevator.setDefaultCommand(ElevatorCommands.setGravity());
        coralPivot.setDefaultCommand(CoralPivotCommands.setDefault());

        swerveDrive.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            swerveDrive.applyRequest(() ->
            drive.withVelocityX(.25*MaxSpeed)
            .withVelocityY(0)
            .withRotationalRate(0)).withTimeout(1.2),
            swerveDrive.applyRequest(() ->
            drive.withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)).withTimeout(10)
        );
    }

    public Elevator getElevator() {
        return elevator;
    }

    public Hang getHang() {
        return hang;
    }

    public CoralPivot getCoralPivot() {
        return coralPivot;
    }

    public Algae getAlgae() {
        return algae;
    }

    public Autons getAutons() {
        return autons;
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }
}
