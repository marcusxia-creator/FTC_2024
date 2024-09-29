package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name= "Test_Zirui", group = "OpMode")
public class Test extends LinearOpMode {
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    public DcMotor intakeMotor;

    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FL_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR_Motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BL_Motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "BR_Motor");

        intakeMotor = hardwareMap.get(DcMotor.class, "In_Motor");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Turn the motor back on when we are done

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        int res = 2;
        int position = Math.round(res * 537.7);

        telemetry.addLine("Robot Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x;
            double rx = gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            int position = intakeMotor.getCurrentPosition();

            double frontLeftMotorPower = (y + x + rx * 0.4) / denominator;
            double frontRightMotorPower = (y - x - rx * 0.4) / denominator;
            double backLeftMotorPower = (y - x + rx * 0.4)/ denominator;
            double backRightMotorPower = (y + x - rx * 0.4) / denominator;

            frontLeftMotor.setPower(frontLeftMotorPower);
            frontRightMotor.setPower(frontRightMotorPower);
            backLeftMotor.setPower(backLeftMotorPower);
            backRightMotor.setPower(backRightMotorPower);

            telemetry.addLine("Robot Initialized");
            telemetry.addData("Front Left Power", frontLeftMotor.getPower());
            telemetry.addData("Front Right Power", frontRightMotor.getPower());
            telemetry.addData("Back Left Power", backLeftMotor.getPower());
            telemetry.addData("Back Right Power", backRightMotor.getPower());
            telemetry.addData("Joystick x", x);
            telemetry.addData("Joystick y", y);
            telemetry.addData("Joystick rx", rx);
            telemetry.addData("Intake Motor position", position);
            telemetry.update();
        }
    }
}