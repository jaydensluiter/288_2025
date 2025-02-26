// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Subsystem;

// public class CommandAlgaeMech implements Subsystem {
//     private final SparkMax AlgaeMotor = new SparkMax(17, MotorType.kBrushless); // Coral arm pivot motor
//     private final Encoder AlgaeEncoder = new Encoder(2, 3); // Encoder for the elevator lift

//     private static final double ENCODER_TICKS_PER_DEGREE = 2048.0 / 360.0; // Translate ticks to degrees
//     private static int PIVOT_TARGET = 0; // Target pivot point
//     private static final double PIVOT_MAX_SPEED = 0.5; // Max speed for the arm

//     /* Subsystem init */
//     public CommandAlgaeMech() {
//         AlgaeEncoder.reset();
//     }
    
//     /* Get pivot angle */
//     public double getPivotAngle() {
//         double PivotAngle = AlgaeEncoder.getDistance() / ENCODER_TICKS_PER_DEGREE;
//         return PivotAngle;
//     }

//     /**
//      * Moves the arm to a specific angle.
//      * @param targetAngle Target angle in degrees.
//      */
//     public void PivotToRot(double targetAngle) {
//         double currentAngle = getPivotAngle();
//         double error = targetAngle - currentAngle;

//         // Simple proportional control (P-Control)
//         double kP = 0.02; // Adjust this constant for your system
//         double speed = kP * error;

//         // Limit the speed to avoid overshooting
//         speed = Math.max(-PIVOT_MAX_SPEED, Math.min(speed, PIVOT_MAX_SPEED));

//         // Run motor until close enough to the target angle
//         while (Math.abs(error) > 1.0) { // 1-degree tolerance
//             AlgaeMotor.set(speed);
//             currentAngle = getPivotAngle();
//             error = targetAngle - currentAngle;
//             speed = kP * error;
//             speed = Math.max(-PIVOT_MAX_SPEED, Math.min(speed, PIVOT_MAX_SPEED));
//         }

//         AlgaeMotor.stopMotor();
//     }

//     public Command PivotToAngle(int targetAngle) {
//         return runOnce(
//             () -> {
//                 PIVOT_TARGET = targetAngle;
//                 PivotToRot(PIVOT_TARGET);
//             });
//     }
// }
