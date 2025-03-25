package frc.robot.commands;

import frc.robot.RobotContainer;

public class CommandsManager {
    private final RobotContainer robotContainer;

    public CommandsManager() {
        robotContainer = new RobotContainer();
    }

    public void createCommands() {
        ElevatorCommands.createCommands(robotContainer);
        AlgaeCommands.createCommands(robotContainer);
        CoralPivotCommands.createCommands(robotContainer);
        SwerveCommands.createCommands(robotContainer);
        HangCommands.createCommands(robotContainer);
    }
}
