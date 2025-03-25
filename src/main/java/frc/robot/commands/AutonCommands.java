package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Autons;
import static edu.wpi.first.wpilibj2.command.Commands.run;

public class AutonCommands {
    private static Autons autons;
    
    public static void createCommands(RobotContainer robotContainer) {
        autons = robotContainer.getAutons();
    }

    public static Command coralPivotAuton() {
        return run(() -> autons.coralPivotAuton());
    }

    public static Command coralPivotAutonStop() {
        return run(() -> autons.coralPivotAutonStop());
    }
}
