package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BasicTeleOps_GW", group = "Linear Opmode")
public class BasicTeleOps extends OpMode {
    public  RobotHardware robot;
    public GamepadEx gamepadCo1;
    public GamepadEx gamepadCo2;
    public RobotDrive robotDrive;
    public FiniteMachineStateArm depositArmDrive;

    @Override
    public void init() {
        robot = new RobotHardware();
        robot.init(hardwareMap); // Initialize hardware in RobotHardware
        gamepadCo1 = new GamepadEx(gamepad2);
        robotDrive = new RobotDrive(robot, gamepadCo1, telemetry); // Pass robot instance to RobotDrive
        robotDrive.init(); // Initialize RobotDrive
        depositArmDrive = new FiniteMachineStateArm(robot, gamepadCo1, telemetry,0.5, .07, 1.0, 500, 1500); // Pass parameters as needed);
        depositArmDrive.init();
        telemetry.addLine("-------------------");
        telemetry.addData("Status"," initialized Motors and Encoder and IMU and Arm Control");
        telemetry.addLine("-------------------");
        telemetry.update();
    }

    @Override
    public void loop() {
        robotDrive.driveLoop(); // Use RobotDrive methods
        depositArmDrive.armLoop();
    }

    public void stop() {;
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.liftMotorLeft.setPower(0);
        robot.liftMotorRight.setPower(0);
        robot.intakeServo.setPosition(0.2);
        telemetry.addData("Status", "Robot stopped");
        telemetry.update();
    }
}
