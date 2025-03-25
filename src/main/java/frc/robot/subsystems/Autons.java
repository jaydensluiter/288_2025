package frc.robot.subsystems;

public class Autons {
    private CoralPivot coralPivot;


    public Autons() {
        coralPivot = new CoralPivot();
    }

    public void coralPivotAuton() {
        coralPivot.set(.3);
    }

    public void coralPivotAutonStop() {
        coralPivot.set(0);
    }
}
