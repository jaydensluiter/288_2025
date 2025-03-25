package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.HangCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.CoralPivotCommands;

public class OperatorController {
    private static CommandPS4Controller controller;

    public OperatorController(int port) {
     controller = new CommandPS4Controller(port);

     configDPadUp();
     configDPadDown();
     configDPadLeft();
     configDPadRight();
     configCircle();
     configSquare();
     configLeftBumper();
     configRightBumper();
     configLeftTrigger();
     configRightTrigger();
     configTriangle();
     configX();
  }

 
  private static void configDPadUp() {
    Trigger dPadUp = controller.povUp();

    dPadUp.onTrue(ElevatorCommands.moveToL4());
  }

  private static void configDPadDown() {
    Trigger dPadDown = controller.povDown();

    dPadDown.onTrue(ElevatorCommands.moveToStow());
  }

  private static void configDPadLeft() {
    Trigger dPadLeft = controller.povLeft();

    dPadLeft.onTrue(ElevatorCommands.moveToL3());
  }

  private static void configDPadRight() {
    Trigger dPadRight = controller.povRight();

    dPadRight.onTrue(ElevatorCommands.moveToLoad());
  }

  private static void configCircle() {
    controller.circle().onTrue(ElevatorCommands.resetEncoder());
  }

  private static void configSquare() {
    controller.square().onTrue(ElevatorCommands.stab());
  }

  private static void configTriangle() {
    controller.triangle().whileTrue(ElevatorCommands.manualMoveUp());
  }

  private static void configX() {
    Trigger X = controller.cross();

    X.whileTrue(ElevatorCommands.manualMoveDown());
  }

  private static void configLeftBumper() {
    Trigger leftBumper = controller.L1();

    leftBumper.whileTrue(HangCommands.moveDown());
  }

  private static void configRightBumper() {
    Trigger rightBumper = controller.R1();

    rightBumper.whileTrue(HangCommands.moveUp());
  }

  private static void configLeftTrigger() {
    Trigger leftTrigger = controller.L2();

    leftTrigger.whileTrue(CoralPivotCommands.moveDown());
  }

  private static void configRightTrigger() {
    Trigger rightTrigger = controller.R2();

    rightTrigger.whileTrue(CoralPivotCommands.moveUp());
  }
}
