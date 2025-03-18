package frc.robot.commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.run;

public class AlgaeCommands {
    private static Algae algae;

    public static void createCommands(RobotContainer robotContainer) {
        algae = robotContainer.getAlgae();
    }

    public static Command moveUp() {
        return run(() -> algae.moveWristUp());
    }

    public static Command moveDown() {
        return run(() -> algae.moveWristDown());
    }

    public static Command intake() {
        return run(() -> algae.intake());
    }

    public static Command outtake() {
        return run(() -> algae.outtake());
    }
}
