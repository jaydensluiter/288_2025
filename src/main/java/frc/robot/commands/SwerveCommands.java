package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.RobotContainer;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveCommands {
    private static SwerveDrive swerveDrive;

    public static void createCommands(RobotContainer robotContainer) {
        swerveDrive = robotContainer.getSwerveDrive();
    }

    public static Command seedFieldCentric() {
        return runOnce(() -> swerveDrive.seedFieldCentric());
    }
}
