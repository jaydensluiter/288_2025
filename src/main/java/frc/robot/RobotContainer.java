// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandCoralPivot;
import frc.robot.subsystems.CommandElevator;
import frc.robot.subsystems.CommandHang;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {

    // Flag to track the current speed mode
    private boolean isSlowSpeed = false;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double SlowSpeed = MaxSpeed * 0.2; // 20% of top speed
    private double speedMultiplier = isSlowSpeed ? slowSpeed : 1.0;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Define the slow speed multiplier
    private static final double slowSpeed = 0.5; // Adjust this value as needed

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Define the button to toggle speed mode
    private final CommandPS4Controller Driver = new CommandPS4Controller(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CommandCoralPivot coralPivot = new CommandCoralPivot();
    public final CommandElevator elevator = new CommandElevator();
    public final CommandHang hang = new CommandHang();

    public RobotContainer() {
        configureBindings();
        
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(-Driver.getLeftY() * MaxSpeed * speedMultiplier) // Drive forward with negative Y (forward)
                    .withVelocityY(-Driver.getLeftX() * MaxSpeed * speedMultiplier) // Drive left with negative X (left)
                    .withRotationalRate(-Driver.getRightX() * MaxAngularRate * speedMultiplier) // Drive counterclockwise with negative X (left)
            )
        );

        elevator.setDefaultCommand(elevator.setGravity());
        coralPivot.setDefaultCommand(coralPivot.setBrake());

        // Toggle speed mode when the square button is pressed
        Driver.square().onTrue(new InstantCommand(() -> {
            isSlowSpeed = !isSlowSpeed; // Toggle the speed mode
        }));

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        Driver.circle().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-Driver.getLeftY(), -Driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        Driver.share().and(Driver.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        Driver.share().and(Driver.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        Driver.options().and(Driver.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        Driver.options().and(Driver.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        Driver.R1().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        operator.povUp().onTrue(coralPivot.setT4Command());
        operator.povRight().onTrue(coralPivot.setT3Command());
        operator.povDown().onTrue(coralPivot.setLoadCommand());

        operator.rightTrigger().whileTrue(hang.Up());
        operator.leftTrigger().whileTrue(hang.Down());
        operator.rightTrigger().onFalse(hang.Stop());
        operator.leftTrigger().onFalse(hang.Stop());

        /* Scoring macros */
        operator.y().onTrue(Commands.parallel(coralPivot.setStowCommand(), elevator.setStowPosition()));
        operator.a().onTrue(Commands.parallel(coralPivot.setLoadCommand(), elevator.setLoadingPosition()));
        operator.b().onTrue(Commands.parallel(coralPivot.setT2Command(), elevator.setT2Position()));
        operator.x().onTrue(Commands.sequence(elevator.setStabPosition(), elevator.setLoadingPosition()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
