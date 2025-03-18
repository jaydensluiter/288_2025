package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Hang;

import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.run;

public class HangCommands {
    private static Hang hang;

    public static void createCommands(RobotContainer robotContainer) {
        hang = robotContainer.getHang();
    }

    public static Command moveUp() {
        return run(() -> hang.moveUp());
    }

    public static Command moveDown() {
        return run(() -> hang.moveDown());
    }
}