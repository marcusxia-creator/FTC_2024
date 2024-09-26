package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Test {
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    public Gamepad gamepad1;

    public Test(HardwareMap hardwaremap) {
        frontLeftMotor = hardwaremap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwaremap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwaremap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwaremap.get(DcMotor.class, "backRightMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    // Method to initialize the gamepad
    public void initGamepad(Gamepad gamepad) {
        this.gamepad1 = gamepad;
    }

    public void drive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftMotorPower = (y + x + rx) / denominator;
        double frontRightMotorPower = (y - x + rx) / denominator;
        double backLeftMotorPower = (y - x - rx) / denominator;
        double backRightMotorPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftMotorPower);
        frontRightMotor.setPower(frontRightMotorPower);
        backLeftMotor.setPower(backLeftMotorPower);
        backRightMotor.setPower(backRightMotorPower);
    }
}
