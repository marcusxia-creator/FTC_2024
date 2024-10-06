package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp (name= "DepositeTest_Zirui", group = "LinearOpMode")
public class DepositeTest extends LinearOpMode {
    /*
    configuration for the test bot hardware
    Motors for chassis
    FL_Motor; BL_motor; FR_Motor; BR_Motor;
    Odometry for chassis
    FL_Motor; BL_motor; FR_Motor;
    Servo for Intake
    Intake_Servo
    Motor for vertical slide
    VS_Motor_1
     */
    public DcMotorEx fDepositeMotor;

    public Gamepad gamepad1;
    @Override
    public void runOpMode() {
        fDepositeMotor = hardwareMap.get(DcMotorEx.class, "VS_Motor_1");

        fDepositeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fDepositeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int CPR = 537;
        int DE_res = 2;
        int DE_rotation_ticks = (DE_res * CPR);

        boolean DepositeMotorExtend = false;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x && !DepositeMotorExtend) {
                fDepositeMotor.setTargetPosition(DE_rotation_ticks);
                fDepositeMotor.setPower(0.1);
                DepositeMotorExtend = true;
            } else if (gamepad1.y && DepositeMotorExtend) {
                fDepositeMotor.setTargetPosition(-DE_rotation_ticks);
                fDepositeMotor.setPower(0.1);
                DepositeMotorExtend = false;
            }

            telemetry.addData("Deposite Slides Extended:", DepositeMotorExtend);
            telemetry.addData("Deposite Extending rotation:", DE_rotation_ticks);
            telemetry.addData("Deposite Motor Position:", fDepositeMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
