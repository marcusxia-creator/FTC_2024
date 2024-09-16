package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FiniteMachineStateArm {
    private final GamepadEx gamepad;
    private final RobotHardware robot;
    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private Telemetry telemetry;

    private final double DEBOUNCE_THRESHOLD = 0.2; // Debouncing threshold for button presses

    public FiniteMachineStateArm(RobotHardware robot, GamepadEx gamepad, Telemetry telemetry, double DUMP_IDLE, double DUMP_DEPOSIT, double DUMP_TIME, int LIFT_LOW, int LIFT_HIGH) {
        this.gamepad = gamepad;
        this.robot = robot;
        this.DUMP_IDLE = DUMP_IDLE;
        this.DUMP_DEPOSIT = DUMP_DEPOSIT;
        this.DUMP_TIME = DUMP_TIME;
        this.LIFT_LOW = LIFT_LOW;
        this.LIFT_HIGH = LIFT_HIGH;
        this.telemetry = telemetry;
    }

    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT
    }

    private LiftState liftState = LiftState.LIFT_START; // Persisting state
    private ElapsedTime liftTimer = new ElapsedTime(); // Timer for controlling dumping time

    final double DUMP_IDLE;   // Idle position for the dump servo
    final double DUMP_DEPOSIT; // Dumping position for the dump servo
    final double DUMP_TIME;   // Time for dumping action in seconds
    final int LIFT_LOW;       // Encoder position for the low position
    final int LIFT_HIGH;      // Encoder position for the high position

    public void init() {
        liftTimer.reset();
        robot.liftMotor.setTargetPosition(LIFT_LOW);
        robot.liftMotor.set(1.0); // Make sure lift motor is on
    }

    public void armLoop() {
        // Display current lift state and telemetry feedback
        telemetry.addData("Lift State", liftState.toString());
        telemetry.update();

        switch (liftState) {
            case LIFT_START:
                // Debounce the button press for starting the lift extend
                if (gamepad.getButton(GamepadKeys.Button.X) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    robot.liftMotor.setTargetPosition(LIFT_HIGH);
                    liftState = LiftState.LIFT_EXTEND;
                }
                break;
            case LIFT_EXTEND:
                // Check if the lift has reached the high position
                if (isLiftAtPosition(LIFT_HIGH)) {
                    robot.intakeServo.setPosition(DUMP_DEPOSIT); // Move servo to dump position
                    liftTimer.reset();
                    liftState = LiftState.LIFT_DUMP;
                }
                break;
            case LIFT_DUMP:
                // Wait for the dump time to pass
                if (liftTimer.seconds() >= DUMP_TIME) {
                    robot.intakeServo.setPosition(DUMP_IDLE); // Reset servo to idle
                    robot.liftMotor.setTargetPosition(LIFT_LOW); // Start retracting the lift
                    liftState = LiftState.LIFT_RETRACT;
                }
                break;
            case LIFT_RETRACT:
                // Check if the lift has reached the low position
                if (isLiftAtPosition(LIFT_LOW)) {
                    robot.liftMotor.set(0); // Stop the motor after reaching the low position
                    liftState = LiftState.LIFT_START;
                }
                break;
            default:
                liftState = LiftState.LIFT_START;
                break;
        }

        // Handle lift cancel action if 'Y' button is pressed
        if (gamepad.getButton(GamepadKeys.Button.Y) && liftState != LiftState.LIFT_START) {
            liftState = LiftState.LIFT_START;
            robot.liftMotor.set(0); // Ensure the motor is stopped
        }
    }

    // Helper method to check if the lift is within the desired position threshold
    private boolean isLiftAtPosition(int targetPosition) {
        return Math.abs(robot.liftMotor.getCurrentPosition() - targetPosition) < 10;
    }
}
