package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Slides_Test_GW", group = "Linear Opmode")
public class SlideTest extends LinearOpMode {
    public DcMotor armMotor;
    private static final double DEBOUNCE_TIME = 0.2; // 200 milliseconds
    private ElapsedTime debounceTimer = new ElapsedTime();
    private boolean buttonPressed = false;

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
        int armUpPosition = 1000;

        // Position of the arm when it's down
        int armDownPosition = 0;

        int armNewPosition =0;

        // Motor CPR
        int CPR = 537;



        // Find a motor in the hardware map named "Arm Motor"
        armMotor = hardwareMap.dcMotor.get("SlideMotor");

        // Reset the motor encoder so that it reads zero ticks
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Sets the starting position of the arm to the down position
        armMotor.setTargetPosition(armDownPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Robot Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // If the A button is pressed, raise the arm
            if (gamepad1.a) {
                armMotor.setTargetPosition(armUpPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
            }

            // If the B button is pressed, lower the arm
            if (gamepad1.b) {
                armMotor.setTargetPosition(armDownPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
            }

            if (gamepad1.x && debounceTimer.seconds() > DEBOUNCE_TIME) {
                armNewPosition +=20;
                armMotor.setTargetPosition(armNewPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
            }

            if (gamepad1.y && debounceTimer.seconds() > DEBOUNCE_TIME) {
                armNewPosition -=20;
                armMotor.setTargetPosition(armNewPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
            }

            // Get the current position of the armMotor
            double position = armMotor.getCurrentPosition();

            // Get the target position of the armMotor
            double desiredPosition = armMotor.getTargetPosition();

            // Show the position of the armMotor on telemetry
            double resolution = position/537.7;
            double angle = resolution/360;
            double angle_normalized = resolution%360;

            telemetry.addData("Encoder Position", position);

            // Show the target position of the armMotor on telemetry
            telemetry.addData("Desired Position", desiredPosition);

            telemetry.addData("Desired Position", desiredPosition);

            telemetry.addData("resolution Position", resolution);

            telemetry.addData("r_angle", angle);

            telemetry.addData("n_angle", angle_normalized);

            telemetry.update();
        }
    }
}
