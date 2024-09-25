package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class TestMovement extends OpMode {
    public Test test;

    @Override
    public void init(){
        test = new Test(hardwareMap);
    }

    @Override
    public void loop(){

        test.RobotDrive(gamepad1);
    }
}
