package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Slides_Test_FMS", group = "Linear Opmode")
public class SlideTestFMS extends LinearOpMode {

    public DcMotorEx slideMotor;
    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;

    public Servo intakeServo;
    private static final double DEBOUNCE_TIME = 0.2; // 200 milliseconds
    private ElapsedTime debounceTimer = new ElapsedTime();
    private boolean buttonPressed = false;

    private double DEBOUNCE_INTERVAL = 0.3;
    final double grabTime = 3;

    public enum SLIDESTATE {
        SLIDE_START,
        SLIDE_EXTEND,
        INTAKE_OUT,
        SLIDE_RETRACT
    }

    private SLIDESTATE SlideState = SLIDESTATE.SLIDE_START; // Persist state
    private ElapsedTime liftTimer = new ElapsedTime(); // Timer for controlling dumping time

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // Position of the arm when it's lifted
        int slideUp = -1450;

        // Position of the arm when it's down
        int slideDown = 20;
        // position of servo
        double intakeOut = 0.2;
        double intakeIn = 0.9;

        double grabPosition = 0.5;

        // Motor CPR
        int CPR = 537;

        //Map in drive motors
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class,"FL_Motor");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class,"FR_Motor");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class,"BL_Motor");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class,"BR_Motor");

        // Find a motor in the hardware map named "VS_Motor_1 Motor"
        slideMotor = hardwareMap.get(DcMotorEx.class,"VS_Motor_1");
        // Find a servo
        intakeServo = hardwareMap.get(Servo.class, "Intake_Servo");

        // Reset the motor encoder so that it reads zero ticks
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //reset lift timer
        liftTimer.reset();

        // Sets the starting position of the arm to the down position
        slideMotor.setTargetPosition(slideDown);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(0.1);

        //initial intakeServo
        intakeServo.setPosition(0);

        //telemetry
        telemetry.addData("Robot Status", "Initialized");
        telemetry.addData("Slide Motor", slideMotor.getCurrentPosition());
        telemetry.addData("Intake Servo", intakeServo.getPosition());
        telemetry.update();

        // press start button to start running mode
        waitForStart();

        while (opModeIsActive()) {
            // Finite state - If the A button is pressed, raise the arm and hold for 20seconds and lower the arm again.
            telemetry.addData("Lift Status", SlideState.toString());
            switch (SlideState) {
                case SLIDE_START:
                    if (gamepad1.x && debounceTimer.seconds() > DEBOUNCE_INTERVAL) {
                        debounceTimer.reset();
                        buttonPressed = true;
                        slideMotor.setTargetPosition(slideUp);
                        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slideMotor.setPower(0.5);
                        SlideState = SLIDESTATE.SLIDE_EXTEND;
                    }
                    else{
                        buttonPressed = false;
                    }
                    break;
                case SLIDE_EXTEND:
                    if(isLeftAtPosition(slideUp)) {
                        intakeServo.setPosition(intakeOut);
                        liftTimer.reset();
                        SlideState = SLIDESTATE.INTAKE_OUT;
                    };
                    break;
                case INTAKE_OUT:
                    if(liftTimer.seconds() >= grabTime){
                        intakeServo.setPosition(intakeIn);
                        SlideState = SLIDESTATE.SLIDE_RETRACT;
                    }
                    break;

                case SLIDE_RETRACT:
                    slideMotor.setTargetPosition(slideDown);
                    slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideMotor.setPower(0.3);
                    SlideState = SLIDESTATE.SLIDE_START;
                    break;
            }

            //return start state
            if ((gamepad1.y) && SlideState != SLIDESTATE.SLIDE_START) {
                SlideState = SLIDESTATE.SLIDE_START;
                slideMotor.setPower(0); // Ensure the motor is stopped
            }
            // Get the current position of the armMotor
            double position = slideMotor.getCurrentPosition();

            // Get the target position of the armMotor
            double desiredPosition = slideMotor.getTargetPosition();

            // Show the position of the armMotor on telemetry
            double resolution = position/537.7;
            double angle = resolution/360;
            double angle_normalized = resolution%360;

            telemetry.addData("Encoder Position", position);

            // Show the target position of the armMotor on telemetry
            telemetry.addData("Desired Position", desiredPosition);

            telemetry.addData("Desired Position", desiredPosition);

            telemetry.addData("resolution Position", resolution);

            telemetry.addData("Servo Position", intakeServo.getPosition());

            telemetry.addData("r_angle", angle);

            telemetry.addData("n_angle", angle_normalized);

            telemetry.addData("Time", liftTimer.seconds());

            telemetry.update();
        }

    }
    // Helper method to check if the lift is within the desired position threshold
    private boolean isLeftAtPosition(int targetPosition) {
        return Math.abs(slideMotor.getCurrentPosition() - targetPosition) <= 10;
    }
}
