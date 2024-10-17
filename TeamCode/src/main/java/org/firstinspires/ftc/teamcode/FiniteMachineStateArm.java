package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FiniteMachineStateArm {
    private final GamepadEx gamepad;
    private final RobotHardware robot;
    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private final TelemetryManager telemetryManager;

    private final double DEBOUNCE_THRESHOLD = 0.2; // Debouncing threshold for button presses

    public FiniteMachineStateArm(RobotHardware robot, GamepadEx gamepad, TelemetryManager telemetryManager, double DUMP_IDLE, double DUMP_DEPOSIT, double DUMP_TIME, double INTAKE_IDLE,double INTAKE_DUMP,int LIFT_LOW, int LIFT_HIGH, double UPLIFT_POWER, double DOWNLIFT_POWER) {
        this.gamepad = gamepad;
        this.robot = robot;
        this.DUMP_IDLE = DUMP_IDLE;
        this.DUMP_DEPOSIT = DUMP_DEPOSIT;
        this.DUMP_TIME = DUMP_TIME;
        this.LIFT_LOW = LIFT_LOW;
        this.LIFT_HIGH = LIFT_HIGH;
        this.telemetryManager = telemetryManager;
        this.UPLIFT_POWER = UPLIFT_POWER;
        this.DOWNLIFT_POWER = DOWNLIFT_POWER;
        this.INTAKE_IDLE = INTAKE_IDLE;
        this.INTAKE_DUMP = INTAKE_DUMP;
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
    final double UPLIFT_POWER;
    final double DOWNLIFT_POWER;
    final double INTAKE_IDLE;
    final double INTAKE_DUMP;

    public void init() {
        liftTimer.reset();
        robot.liftMotorLeft.setTargetPosition(LIFT_LOW);
        robot.liftMotorRight.setTargetPosition(LIFT_LOW);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(0.1); // Make sure lift motor is on
        robot.liftMotorRight.setPower(0.1);
        robot.IntakeArmServo.setPosition(DUMP_IDLE);
        robot.IntakeServo.setPosition(INTAKE_IDLE);

        telemetryManager.update("Lift Motor Left Position", robot.liftMotorLeft.getCurrentPosition());
        telemetryManager.update("Lift Motor Right Position", robot.liftMotorRight.getCurrentPosition());
    }

    public void armLoop() {
        // Display current lift state and telemetry feedback
        telemetryManager.update("Lift State", liftState.toString());
        telemetryManager.update("Servo position", robot.IntakeServo.getPosition());
        telemetryManager.update("Servo position", robot.IntakeArmServo.getPosition());
        telemetryManager.update("lift motor", robot.liftMotorLeft.getCurrentPosition());
        telemetryManager.update("lift motor", robot.liftMotorLeft.getTargetPosition());
        telemetryManager.update("Right motor", robot.liftMotorRight.getCurrentPosition());
        telemetryManager.update("Right motor", robot.liftMotorRight.getTargetPosition());

        switch (liftState) {
            case LIFT_START:
                // Debounce the button press for starting the lift extend
                if (gamepad.getButton(GamepadKeys.Button.X) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    robot.liftMotorLeft.setTargetPosition(LIFT_HIGH);
                    robot.liftMotorRight.setTargetPosition(LIFT_HIGH);
                    robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorLeft.setPower(UPLIFT_POWER);
                    robot.liftMotorRight.setPower(UPLIFT_POWER);
                    liftState = LiftState.LIFT_EXTEND;
                }
                break;
            case LIFT_EXTEND:
                // Check if the lift has reached the high position
                if (isLiftAtPosition(LIFT_HIGH)) {
                    liftTimer.reset();
                    robot.IntakeArmServo.setPosition(DUMP_DEPOSIT); // Move servo to dump position
                    if (liftTimer.seconds() >= DUMP_TIME) {
                        robot.IntakeServo.setPosition(INTAKE_DUMP);
                        liftState = LiftState.LIFT_DUMP;
                    }
                }
                break;
            case LIFT_DUMP:
                // Wait for the dump time to pass
                if (liftTimer.seconds() >= DUMP_TIME) {
                    robot.IntakeArmServo.setPosition(DUMP_IDLE); // Reset servo to idle
                    robot.IntakeServo.setPosition(INTAKE_IDLE);
                }
                if(liftTimer.seconds()>= DUMP_TIME + 1){
                    liftState = LiftState.LIFT_RETRACT;
                }
                break;
            case LIFT_RETRACT:
                // Check if the lift has reached the low position
                robot.liftMotorLeft.setTargetPosition(LIFT_LOW); // Start retracting the lift
                robot.liftMotorRight.setTargetPosition(LIFT_LOW); // Start retracting the lift
                robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotorLeft.setPower(DOWNLIFT_POWER);
                robot.liftMotorRight.setPower(DOWNLIFT_POWER);
                if (isLiftAtPosition(LIFT_LOW)) {
                    robot.liftMotorLeft.setPower(0); // Stop the motor after reaching the low position
                    robot.liftMotorRight.setPower(0);
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
            robot.liftMotorLeft.setPower(0); // Ensure the motor is stopped
            robot.liftMotorRight.setPower(0);
        }
    }

    // Helper method to check if the lift is within the desired position threshold
    private boolean isLiftAtPosition(int targetPosition) {
        return Math.abs(robot.liftMotorLeft.getCurrentPosition() - targetPosition) < 5 && Math.abs(robot.liftMotorRight.getCurrentPosition() - targetPosition) < 5;
    }
}
