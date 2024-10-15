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
    public FiniteMachineStateArm armDrive;

    @Override
    public void init() {
        robot = new RobotHardware();
        robot.init(hardwareMap); // Initialize hardware in RobotHardware
        gamepadCo1 = new GamepadEx(gamepad2);
        robotDrive = new RobotDrive(robot, gamepadCo1, telemetry); // Pass robot instance to RobotDrive
        robotDrive.init(); // Initialize RobotDrive
        armDrive = new FiniteMachineStateArm(robot, gamepadCo1, telemetry,0.5, .07, 1.0, 500, 900); // Pass parameters as needed);
        armDrive.init();
        telemetry.addLine("-------------------");
        telemetry.addData("Status"," initialized Motors and Encoder and IMU and Arm Control");
        telemetry.addLine("-------------------");
        telemetry.update();
    }

    @Override
    public void loop() {
        robotDrive.driveLoop(); // Use RobotDrive methods
        armDrive.armLoop();
    }

    public void stop() {;
        robot.frontLeftMotor.set(0);
        robot.frontRightMotor.set(0);
        robot.backLeftMotor.set(0);
        robot.backRightMotor.set(0);
        robot.liftMotorLeft.setPower(0);
        //robot.verticalSlideMotorRight.set(0);
        robot.intakeServo.setPosition(0.2);
        telemetry.addData("Status", "Robot stopped");
        telemetry.update();
    }
}
