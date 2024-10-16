package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOps_Mecanum_FMS", group = "OpMode")
public class BasicTeleOps extends OpMode {
    public  RobotHardware robot;
    public GamepadEx gamepadCo1;
    public GamepadEx gamepadCo2;
    public RobotDrive robotDrive;
    public FiniteMachineStateArm depositArmDrive;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        robot = new RobotHardware();
        robot.init(hardwareMap); // Initialize hardware in RobotHardware
        gamepadCo1 = new GamepadEx(gamepad2);
        robotDrive = new RobotDrive(robot, gamepadCo1, telemetry,0.5); // Pass robot instance to RobotDrive
        robotDrive.init(); // Initialize RobotDrive
        depositArmDrive = new FiniteMachineStateArm(robot, gamepadCo1, telemetry,0.5, .07, 1.0, 500, 1500); // Pass parameters as needed);
        depositArmDrive.init();
        dashboard = FtcDashboard.getInstance();
        telemetry.addLine("-------------------");
        telemetry.addData("Status"," initialized Motors and Encoder and IMU and Arm Control");
        telemetry.addLine("-------------------");
        telemetry.addData("slide right motor",robot.liftMotorRight.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        robotDrive.driveLoop(); // Use RobotDrive methods
        depositArmDrive.armLoop();

        // Real-time telemetry data to Driver Station
        telemetry.addData("Front Left Motor Power", robot.frontLeftMotor.getPower());
        telemetry.addData("Front Right Motor Power", robot.frontRightMotor.getPower());
        telemetry.addData("Back Left Motor Power", robot.backLeftMotor.getPower());
        telemetry.addData("Back Right Motor Power", robot.backRightMotor.getPower());

        telemetry.addData("Lift Motor Left Position", robot.liftMotorLeft.getCurrentPosition());
        telemetry.addData("Lift Motor Right Position", robot.liftMotorRight.getCurrentPosition());

        // Update telemetry for the driver station
        telemetry.update();

        //FTC DashBoard telemetry packet
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Front Left Motor Power", robot.frontLeftMotor.getPower());
        packet.put("Front Right Motor Power", robot.frontRightMotor.getPower());
        packet.put("Back Left Motor Power", robot.backLeftMotor.getPower());
        packet.put("Back Right Motor Power", robot.backRightMotor.getPower());
        packet.put("Lift Motor Left Position", robot.liftMotorLeft.getCurrentPosition());
        packet.put("Lift Motor Right Position", robot.liftMotorRight.getCurrentPosition());
        packet.put("IMU Heading", robot.getIMUHeading()); // Assuming there's a method to get IMU heading

        dashboard.sentTelemetryPacket(packet);
    }

    public void stop() {;
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.liftMotorLeft.setPower(0);
        robot.liftMotorRight.setPower(0);
        robot.IntakeServo.setPosition(0.2);
        telemetry.addData("Status", "Robot stopped");
        telemetry.update();
    }
}
