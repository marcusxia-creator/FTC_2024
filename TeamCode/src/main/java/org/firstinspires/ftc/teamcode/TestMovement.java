package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name= "TestZirui", group = "OpMode")
public class TestMovement extends OpMode {
    //delcare the robot drive (Test class)
    private Test robotDrive;

    @Override
    public void init(){
        // instantiate the test class.
        //robotDrive = new Test(hardwareMap);
        // initialize gamepad
        //robotDrive.initGamepad(gamepad1);
        //telemetry the status of robot.
        //telemetry.addLine("Robot Initialized");
        //telemetry.update();
    }

    @Override
    public void loop(){
        //call the drive method to drive the robot
        //robotDrive.drive();
        // Add telemetry for debugging
        //telemetry.addData("Front Left Power", robotDrive.frontLeftMotor.getPower());
        //telemetry.addData("Front Right Power", robotDrive.frontRightMotor.getPower());
        //telemetry.addData("Back Left Power", robotDrive.backLeftMotor.getPower());
        //telemetry.addData("Back Right Power", robotDrive.backRightMotor.getPower());
        //telemetry.update();

    }
}
