package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestMovement extends OpMode {
    public Test test;

    @Override
    public void init(){
        test.RobotHardware(hardwareMap);
    }

    @Override
    public void loop(){
        test.RobotDrive();
    }
}
