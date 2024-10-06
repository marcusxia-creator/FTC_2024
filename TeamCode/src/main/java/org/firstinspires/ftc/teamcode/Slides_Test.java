package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp (name= "Slides_Test_Zirui", group = "LinearOpMode")
public class Slides_Test extends LinearOpMode {
    /*
    configuration for the test bot
    Motors for chassis
    FL_Motor; BL_motor; FR_Motor; BR_Motor;
    Odometry for chassis
    FL_Motor; BL_motor; FR_Motor;
    Servo for Intake
    Intake_Servo
    Motor for vertical slide
    VS_Motor_1
    */
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    public DcMotor IntakeMotor;

    public DcMotor fDepositeMotor;
    public DcMotor sDepositeMotor;

    public Gamepad gamepad1;

    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FL_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR_Motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BL_Motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "BR_Motor");

        IntakeMotor = hardwareMap.get(DcMotor.class, "1IN_Motor");

        fDepositeMotor = hardwareMap.get(DcMotor.class, "VS_Motor_1");
        sDepositeMotor = hardwareMap.get(DcMotor.class, "2DE_Motor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fDepositeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fDepositeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sDepositeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sDepositeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int ticks = 537;
        int IN_res = 2;
        int DE_res = 2;
        int IN_rotation = (IN_res * ticks);
        int DE_rotation = (DE_res * ticks);

        boolean IntakeMotorExtend = false;
        boolean DepositeMotorExtend = false;

        double motorMaxSpeed = 0.4;

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftMotorPower = (y + x + rx * motorMaxSpeed) / denominator;
            double frontRightMotorPower = (y - x - rx * motorMaxSpeed) / denominator;
            double backLeftMotorPower = (y - x + rx * motorMaxSpeed) / denominator;
            double backRightMotorPower = (y + x - rx * motorMaxSpeed) / denominator;

            frontLeftMotor.setPower(frontLeftMotorPower);
            frontRightMotor.setPower(frontRightMotorPower);
            backLeftMotor.setPower(backLeftMotorPower);
            backRightMotor.setPower(backRightMotorPower);

            //int IN_position = IntakeMotor.getCurrentPosition();
            //int DE_position = fDepositeMotor.getCurrentPosition();

            if (gamepad1.a && !IntakeMotorExtend) {
                IntakeMotor.setTargetPosition(IN_rotation);
                IntakeMotor.setPower(0.1);
                IntakeMotorExtend = true;
            }
            else if(gamepad1.b && IntakeMotorExtend) {
                IntakeMotor.setTargetPosition(-IN_rotation);
                IntakeMotor.setPower(0.1);
                IntakeMotorExtend = false;
            }
            else if (gamepad1.x && !DepositeMotorExtend) {
                fDepositeMotor.setTargetPosition(DE_rotation);
                fDepositeMotor.setPower(0.1);
                sDepositeMotor.setTargetPosition(DE_rotation);
                sDepositeMotor.setPower(0.1);
                DepositeMotorExtend = true;
            }
            else if (gamepad1.y && DepositeMotorExtend) {
                fDepositeMotor.setTargetPosition(-DE_rotation);
                fDepositeMotor.setPower(0.1);
                sDepositeMotor.setTargetPosition(-DE_rotation);
                sDepositeMotor.setPower(0.1);
                DepositeMotorExtend = false;
            }

            telemetry.addData("Front Left Motor Power:", frontLeftMotorPower);
            telemetry.addData("Front Right Motor Power:", frontRightMotorPower);
            telemetry.addData("Back Left Motor Power:", backLeftMotorPower);
            telemetry.addData("Back Right Motor Power:", backRightMotorPower);
            telemetry.addData("Intake Slides Extended:", IntakeMotorExtend);
            //telemetry.addData("Intake Extending rotation:", IN_res);
            telemetry.addData("Intake Motor Position:", IntakeMotor.getCurrentPosition());
            telemetry.addData("Deposite Slides Extended:", DepositeMotorExtend);
            //telemetry.addData("Deposite Extending rotation:", DE_rotation);
            telemetry.addData("Deposite Motor Position:", fDepositeMotor.getCurrentPosition());
            telemetry.addData("Joystick x:", x);
            telemetry.addData("Joystick y:", y);
            telemetry.addData("Joystick rx:", rx);
            telemetry.update();
        }
    }
}
