package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralPivot;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import edu.wpi.first.wpilibj2.command.Command;

public final class CoralPivotCommands {
    private static CoralPivot coralPivot;

    public static void createCommands(RobotContainer robotContainer) {
        coralPivot = robotContainer.getCoralPivot();
    }

    public static Command moveUp() {
        return run(() -> coralPivot.moveUp());
    }

    public static Command moveDown() {
        return run(() -> coralPivot.moveDown());
    }

    public static Command setDefault() {
        return run(() -> coralPivot.setDefault());
    }
}


