package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.wpilibj2.command.Commands.run;

public final class ElevatorCommands {
    private static Elevator elevator;

    public static void createCommands(RobotContainer robotContainer) {
        elevator = robotContainer.getElevator();
    }

    public static Command moveToL3() {
        return elevator.moveToSetPositionCommand(Constants.Elevator.kL3);
    }

    public static Command moveToL4() {
        return elevator.moveToSetPositionCommand(Constants.Elevator.kL3);
    }

    public static Command stab() {
        return Commands.sequence(
            elevator.moveToSetPositionCommand(Constants.Elevator.kStab), 
            elevator.moveToSetPositionCommand(Constants.Elevator.kLoad)
        );
    }

    public static Command moveToLoad() {
        return elevator.moveToSetPositionCommand(Constants.Elevator.kLoad);
    }

    public static Command moveToStow() {
        return elevator.moveToSetPositionCommand(Constants.Elevator.kStow);
    }

    public static Command resetEncoder() {
        return run(() -> elevator.resetEncoder());
    }

    public static Command setGravity() {
        return run(() -> elevator.setGravity());
    }

    public static Command manualMoveUp() {
        return run(() -> elevator.manualMoveUp());
    }

    public static Command manualMoveDown() {
        return run(() -> elevator.manualMoveDown());
    }
}
