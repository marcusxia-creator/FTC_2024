package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Slides_Test_GW", group = "Linear Opmode")
public class SlideTest extends LinearOpMode {
    public DcMotor armMotorLeft;
    public DcMotor armMotorRight;
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
        armMotorLeft = hardwareMap.dcMotor.get("VS_Motor_Left");
        armMotorRight = hardwareMap.dcMotor.get("VS_Motor_Right");

        // Reset the motor encoder so that it reads zero ticks
        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorLeft.setDirection(DcMotor.Direction.REVERSE);


        // Sets the starting position of the arm to the down position
        armMotorLeft.setTargetPosition(armDownPosition);
        armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorRight.setTargetPosition(armDownPosition);
        armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Robot Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // If the A button is pressed, raise the arm
            if (gamepad1.a) {
                armMotorLeft.setTargetPosition(armUpPosition);
                armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotorRight.setTargetPosition(armUpPosition);
                armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotorLeft.setPower(0.5);
                armMotorRight.setPower(0.5);
            }

            // If the B button is pressed, lower the arm
            if (gamepad1.b) {
                armMotorLeft.setTargetPosition(armDownPosition);
                armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotorRight.setTargetPosition(armDownPosition);
                armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotorLeft.setPower(0.5);
                armMotorLeft.setPower(0.5);
            }

            if (gamepad1.x && debounceTimer.seconds() > DEBOUNCE_TIME) {
                armNewPosition =armMotorLeft.getCurrentPosition() + 20;
                armMotorLeft.setTargetPosition(armNewPosition);
                armMotorRight.setTargetPosition(armNewPosition);
                armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotorLeft.setPower(0.5);
                armMotorRight.setPower(0.5);
            }

            if (gamepad1.y && debounceTimer.seconds() > DEBOUNCE_TIME) {
                armNewPosition = armMotorLeft.getCurrentPosition() - 20;
                armMotorLeft.setTargetPosition(armNewPosition);
                armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotorRight.setTargetPosition(armNewPosition);
                armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotorLeft.setPower(0.5);
                armMotorRight.setPower(0.5);
            }

            // Get the current position of the armMotor
            double Rposition = armMotorRight.getCurrentPosition();
            double Lposition = armMotorLeft.getCurrentPosition();

            // Get the target position of the armMotor
            double RdesiredPosition = armMotorRight.getTargetPosition();
            double LdesiredPosition = armMotorLeft.getTargetPosition();

            // Show the position of the armMotor on telemetry
            double resolution = Rposition/537.7;
            double angle = resolution/360;
            double angle_normalized = resolution%360;

            telemetry.addData("Right Encoder Position", Rposition);
            telemetry.addData("Left Encoder Position", Lposition);

            // Show the target position of the armMotor on telemetry
            telemetry.addData("Right Desired Position", RdesiredPosition);
            telemetry.addData("Left Desired Position", LdesiredPosition);


            telemetry.addData("resolution Position", resolution);

            telemetry.addData("r_angle", angle);

            telemetry.addData("n_angle", angle_normalized);

            telemetry.update();
        }
    }//end main loop
}//end SlideTest class