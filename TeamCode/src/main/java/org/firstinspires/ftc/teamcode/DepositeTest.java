package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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


    @Override
    public void runOpMode() {
        fDepositeMotor = hardwareMap.get(DcMotorEx.class, "VS_Motor_1");

        fDepositeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fDepositeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int CPR = 537;
        int DE_res = 2;
        int DE_rotation_ticks = (DE_res * CPR);

        boolean DepositeMotorExtend = false;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x && !DepositeMotorExtend) {
                if (!AtPosition(DE_rotation_ticks)){
                    fDepositeMotor.setTargetPosition(DE_rotation_ticks);
                    fDepositeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    fDepositeMotor.setPower(0.4);
                }
                else {
                    DepositeMotorExtend = true;
                    fDepositeMotor.setPower(0);
                }
            }
            else if (gamepad1.y && DepositeMotorExtend) {
                if (!AtPosition(DE_rotation_ticks)) {
                    fDepositeMotor.setTargetPosition(-DE_rotation_ticks);
                    fDepositeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    fDepositeMotor.setPower(0.4);
                }
                else {
                    DepositeMotorExtend = false;
                    //test if set power to 0 is needed
                    fDepositeMotor.setPower(0);
                }
            }

            telemetry.addData("Deposite Slides Extended:", DepositeMotorExtend);
            telemetry.addData("Deposite Extending rotation:", DE_rotation_ticks);
            telemetry.addData("Deposite Motor Position:", fDepositeMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    private boolean AtPosition(int targetPosition) {
        return (Math.abs(targetPosition) - Math.abs(fDepositeMotor.getCurrentPosition()) < 5);
    }
}
