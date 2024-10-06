package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VslideControl {
    /*
    config gamepad 2 for slide control
     X button -

     */
    private final RobotHardware robot;
    private final GamepadEx gamepad;
    private final Telemetry telemetry;
    private ElapsedTime debouncerTimer = new ElapsedTime();
    private ElapsedTime liftTimer = new ElapsedTime(); // Timer for controlling intake running time

    //config button time
    private boolean buttonPressed = false;
    private double DEBOUNCE_INTERVAL = 0.3; // seconds interval
    // config grabtime
    private final double grabTime = 3;

    // config slide position and intake position
    int slideUp = -300;

    // Position of the arm when it's down
    int slideDown = 20;
    // position of servo
    double intakeOut = 0;
    double intakeIn = 90;
    double grabPosition = 0.5;
    // Motor CPR
    int CPR = 537;

    //declare slide state
    public enum SLIDESTATE {
        SLIDE_START,
        SLIDE_EXTEND,
        INTAKE_OUT,
        SLIDE_RETRACT
    }

    //default slidestate status
    private SLIDESTATE SlideState = SLIDESTATE.SLIDE_START; // Persist state

    public VslideControl(RobotHardware robot, GamepadEx gamepad, Telemetry telemetry) {
        this.robot = robot;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }

    public void VslideInitial(){
        //reset lift timer
        liftTimer.reset();
        // reset the encoder
        robot.verticalSlideMotor1.resetEncoder();

// the current position of the motor
        int pos = robot.verticalSlideMotor1.getCurrentPosition();

        // Sets the starting position of the arm to the down position
        robot.verticalSlideMotor1.setTargetPosition(pos);
        robot.verticalSlideMotor1.setRunMode(Motor.RunMode.PositionControl);
        robot.verticalSlideMotor1.set(0.1);

        //initial intakeServo
        robot.intakeServo.turnToAngle(90);

        //telemetry
        telemetry.addData("Robot Status", "Initialized");
        telemetry.addData("Slide Motor", robot.verticalSlideMotor1.getCurrentPosition());
        telemetry.addData("Intake Servo", robot.intakeServo.getPosition());
        telemetry.update();
    }
    public void VslideRun(){
        telemetry.addData("Lift Status", SlideState.toString());
        switch (SlideState) {
            case SLIDE_START:
                if (gamepad.getButton(GamepadKeys.Button.X) && debouncerTimer.seconds() > DEBOUNCE_INTERVAL) {
                    debouncerTimer.reset();
                    buttonPressed = true;
                    robot.verticalSlideMotor1.setTargetPosition(slideUp);
                    robot.verticalSlideMotor1.setRunMode(Motor.RunMode.PositionControl);
                    robot.verticalSlideMotor1.set(0.5);
                    SlideState = SLIDESTATE.SLIDE_EXTEND;
                }
                else{
                    buttonPressed = false;
                }
                break;
            case SLIDE_EXTEND:
                if(isLeftAtPosition(slideUp)) {
                    robot.intakeServo.setPosition(intakeOut);
                    liftTimer.reset();
                    SlideState = SLIDESTATE.INTAKE_OUT;
                };
                break;
            case INTAKE_OUT:
                if(liftTimer.seconds() >= grabTime){
                    robot.intakeServo.setPosition(intakeIn);
                    SlideState = SLIDESTATE.SLIDE_RETRACT;
                }
                break;

            case SLIDE_RETRACT:
                robot.verticalSlideMotor1.setTargetPosition(slideDown);;
                robot.verticalSlideMotor1.set(0.3);
                SlideState = SLIDESTATE.SLIDE_START;
                break;
        }

        //return start state
        if ((gamepad.getButton(GamepadKeys.Button.Y)) && SlideState != SLIDESTATE.SLIDE_START) {
            SlideState = SLIDESTATE.SLIDE_START;
            robot.verticalSlideMotor1.set(0); // Ensure the motor is stopped
        }
        // Get the current position of the armMotor
        double position = robot.verticalSlideMotor1.getCurrentPosition();

        // Get the target position of the armMotor
        double desiredPosition = robot.verticalSlideMotor1.getDistance();

        // Show the position of the armMotor on telemetry
        double resolution = position/CPR;
        double angle = resolution/360;
        double angle_normalized = resolution%360;

        telemetry.addData("Encoder Position","%.2f", position);

        // Show the target position of the armMotor on telemetry
        telemetry.addData("Desired Position", "%.2f",desiredPosition);

        telemetry.addData("resolution Position", "%.2f", resolution);

        telemetry.addData("Servo Position", "%.2f",robot.intakeServo.getPosition());

        telemetry.addData("r_angle", "%.2f", angle);

        telemetry.addData("n_angle", "%.2f", angle_normalized);

        telemetry.addData("Time", "%.2f",liftTimer.seconds());

        telemetry.update();
    }

    // Helper method to check if the lift is within the desired position threshold
    private boolean isLeftAtPosition(int targetPosition) {
        return Math.abs(robot.verticalSlideMotor1.getCurrentPosition() - targetPosition) <= 10;
    }
}

