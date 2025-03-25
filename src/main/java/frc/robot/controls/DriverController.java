package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlgaeCommands;
import frc.robot.commands.SwerveCommands;

public class DriverController {
    private static CommandPS4Controller controller;

    public DriverController(int port) {
        controller = new CommandPS4Controller(port);

        configLeftTrigger();
        configRightTrigger();
        configRightBumper();
        configSquare();
        configX();
    }

    public static void configLeftTrigger() {
        Trigger leftTrigger = controller.L2();

        leftTrigger.whileTrue(AlgaeCommands.moveDown());
        leftTrigger.onFalse(AlgaeCommands.stopWrist());
    }

    public static void configRightTrigger() {
        Trigger rightTrigger = controller.R2();

        rightTrigger.whileTrue(AlgaeCommands.moveUp());
        rightTrigger.onFalse(AlgaeCommands.stopWrist());
    }

    public static void configRightBumper() {
        Trigger rightBumper = controller.R1();

        rightBumper.onTrue(SwerveCommands.seedFieldCentric());
    }

    public static void configSquare() {
        controller.square().whileTrue(AlgaeCommands.intake());
        controller.square().onFalse(AlgaeCommands.stopIntake());
    }

    public static void configX() {
        Trigger X = controller.cross();

        X.whileTrue(AlgaeCommands.outtake());
        X.onFalse(AlgaeCommands.stopIntake());
    }
}
