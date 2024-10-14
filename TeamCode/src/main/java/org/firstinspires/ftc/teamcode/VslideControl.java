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

    // config slide position
    int slideUp = 1000;
    int slideMid = 2000;
    // Position of the arm when it's down
    int slideDown = 10;

    // config servo position of servo
    double intakeOut = 90;
    double intakeIn = 120;
    double grabPosition = 0.5;
    // Motor CPR
    int CPR = 537;

    //declare slide state
    public enum SLIDESTATE {
        SLIDE_START,
        SLIDE_MID,
        INTAKE_HIGH,
        SLID_HIGH,
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
        robot.verticalSlideMotorLeft.resetEncoder();
        robot.verticalSlideMotorRight.resetEncoder();

// the current position of the motor
        int pos = robot.verticalSlideMotorLeft.getCurrentPosition();

        // Sets the starting position of the arm to the down position
        robot.verticalSlideMotorLeft.setTargetPosition(pos);
        robot.verticalSlideMotorRight.setTargetPosition(pos);
        robot.verticalSlideMotorLeft.setRunMode(Motor.RunMode.PositionControl);
        robot.verticalSlideMotorRight.setRunMode(Motor.RunMode.PositionControl);
        robot.verticalSlideMotorLeft.set(0.1);
        robot.verticalSlideMotorRight.set(0.1);

        robot.intakeArmServo.setPosition(0.5);
        robot.intakeServo.setPosition(0.5);

        //initial intakeServo
        //robot.intakeServo.turnToAngle(180);

        //telemetry
        telemetry.addData("Robot Status", "Initialized");
        telemetry.addData("Slide Motor Left", robot.verticalSlideMotorLeft.getCurrentPosition());
        telemetry.addData("Slide Motor Right", robot.verticalSlideMotorRight.getCurrentPosition());
        //telemetry.addData("Intake Servo", robot.intakeServo.getPosition());
        telemetry.update();
    }
    public void VslideRun(){
        telemetry.addData("Lift Status", SlideState.toString());
        switch (SlideState) {
            case SLIDE_START:
                if (gamepad.getButton(GamepadKeys.Button.X) && debouncerTimer.seconds() > DEBOUNCE_INTERVAL) {
                    debouncerTimer.reset();
                    buttonPressed = true;
                    robot.verticalSlideMotorLeft.setTargetPosition(slideUp);
                    robot.verticalSlideMotorRight.setTargetPosition(slideUp);
                    robot.verticalSlideMotorLeft.setRunMode(Motor.RunMode.PositionControl);
                    robot.verticalSlideMotorRight.setRunMode(Motor.RunMode.PositionControl);
                    robot.verticalSlideMotorLeft.set(0.5);
                    robot.verticalSlideMotorRight.set(0.5);
                    SlideState = SLIDESTATE.SLIDE_MID;
                }
                else{
                    buttonPressed = false;

                }
                break;
            case SLIDE_MID:
                if(isLeftAtPosition(slideUp)) {
                    //robot.intakeServo.setPosition(intakeOut);
                    liftTimer.reset();
                    if(liftTimer.seconds() >= grabTime) {
                        robot.intakeArmServo.setPosition(0.2);
                        SlideState = SLIDESTATE.SLID_HIGH;
                    }
                };
                break;
            case SLID_HIGH:
                robot.verticalSlideMotorLeft.setTargetPosition(slideMid);
                robot.verticalSlideMotorRight.setTargetPosition(slideMid);
                robot.verticalSlideMotorLeft.setRunMode(Motor.RunMode.PositionControl);
                robot.verticalSlideMotorRight.setRunMode(Motor.RunMode.PositionControl);
                robot.verticalSlideMotorLeft.set(0.5);
                robot.verticalSlideMotorRight.set(0.5);
                if (isLeftAtPosition(slideMid)){
                    liftTimer.reset();
                    if(liftTimer.seconds() >= grabTime){
                        //robot.intakeServo.setPosition(intakeIn);
                        SlideState = SLIDESTATE.SLIDE_RETRACT;
                    }
                }
                break;
            case SLIDE_RETRACT:
                robot.verticalSlideMotorLeft.setTargetPosition(slideDown);
                robot.verticalSlideMotorRight.setTargetPosition(slideDown);
                robot.verticalSlideMotorLeft.setRunMode(Motor.RunMode.PositionControl);
                robot.verticalSlideMotorRight.setRunMode(Motor.RunMode.PositionControl);
                robot.verticalSlideMotorLeft.set(0.5);
                robot.verticalSlideMotorRight.set(0.5);
                SlideState = SLIDESTATE.SLIDE_START;
                break;
        }

        //return start state
        if ((gamepad.getButton(GamepadKeys.Button.Y)) && SlideState != SLIDESTATE.SLIDE_START && debouncerTimer.seconds() > DEBOUNCE_INTERVAL) {
            debouncerTimer.reset();
            SlideState = SLIDESTATE.SLIDE_START;
            robot.verticalSlideMotorRight.set(0);
            robot.verticalSlideMotorLeft.set(0);// Ensure the motor is stopped
        }
        // Get the current position of the armMotor
        double position = robot.verticalSlideMotorRight.getCurrentPosition();

        // Get the target position of the armMotor
        double desiredPosition = robot.verticalSlideMotorRight.getDistance();

        // Show the position of the armMotor on telemetry
        double resolution = position/CPR;
        double angle = resolution*360;
        double angle_normalized = resolution%360;

        telemetry.addData("Encoder Position","%.2f", position);

        // Show the target position of the armMotor on telemetry
        telemetry.addData("Desired Position", "%.2f",desiredPosition);

        telemetry.addData("resolution Position", "%.2f", resolution);

        //telemetry.addData("Servo Position", "%.2f",robot.intakeServo.getPosition());

        telemetry.addData("r_angle", "%.2f", angle);

        telemetry.addData("n_angle", "%.2f", angle_normalized);

        telemetry.addData("Time", "%.2f",liftTimer.seconds());

        telemetry.update();
    }// end of Vslide control method

    // Helper method to check if the lift is within the desired position threshold
    private boolean isLeftAtPosition(int targetPosition) {
        return Math.abs(robot.verticalSlideMotorLeft.getCurrentPosition() - targetPosition) <= 10 && Math.abs(robot.verticalSlideMotorRight.getCurrentPosition() - targetPosition) <= 10;
    }
}//end of class

