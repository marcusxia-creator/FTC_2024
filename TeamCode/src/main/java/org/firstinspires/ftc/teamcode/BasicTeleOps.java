package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BasicTeleOps_GW", group = "Linear Opmode")
public class BasicTeleOps extends OpMode {
    public  RobotHardware robot;
    public GamepadEx gamepad;
    public RobotDrive robotDrive;

    @Override
    public void init() {
        robot = new RobotHardware();
        robot.init(hardwareMap); // Initialize hardware in RobotHardware
        gamepad = new GamepadEx(gamepad2);
        robotDrive = new RobotDrive(robot, gamepad, telemetry); // Pass robot instance to RobotDrive
        robotDrive.init(); // Initialize RobotDrive
        telemetry.addLine("-------------------");
        telemetry.addData("Status"," initialized Motors and Encoder and IMU");
        telemetry.addLine("-------------------");
        telemetry.update();
    }

    @Override
    public void loop() {

        robotDrive.driveLoop(); // Use RobotDrive methods

    }
}
