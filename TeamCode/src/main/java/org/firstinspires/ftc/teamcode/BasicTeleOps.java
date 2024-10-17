package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "TeleOps_Mecanum_FMS", group = "OpMode")
public class BasicTeleOps extends OpMode {
    public RobotHardware robot;
    public GamepadEx gamepadCo1;
    public GamepadEx gamepadCo2;
    public RobotDrive robotDrive;
    public FiniteMachineStateArm depositArmDrive;
    private FtcDashboard dashboard;
    private TelemetryManager telemetryManager;

    public static double powerFactor = 0.5;
    public static double depArm_Idle = 0.5;
    public static double demArm_Drop = 0.05;
    public static double dropTime = 1.0;
    public static int downLiftpos = 200;
    public static int upLiftpos = 1500;

    public static double upLiftPower = 0.4;
    public static double downLiftPower = 0.3;
    
    @Override
    public void init() {
        telemetryManager = new TelemetryManager(telemetry);
        robot = new RobotHardware();
        robot.init(hardwareMap); // Initialize hardware in RobotHardware
        gamepadCo1 = new GamepadEx(gamepad2);
        robotDrive = new RobotDrive(robot, gamepadCo1, telemetryManager,powerFactor); // Pass robot instance to RobotDrive
        robotDrive.init(); // Initialize RobotDrive
        depositArmDrive = new FiniteMachineStateArm(robot, gamepadCo1, telemetryManager,depArm_Idle, demArm_Drop, dropTime, downLiftpos, upLiftpos, upLiftPower,downLiftPower); // Pass parameters as needed);
        depositArmDrive.init();
        dashboard = FtcDashboard.getInstance();
        telemetry.addLine("-------------------");
        telemetryManager.update("Status"," initialized Motors and Encoder and IMU and Arm Control");
        telemetry.addLine("-------------------");
    }

    @Override
    public void loop() {
        robotDrive.driveLoop(); // Use RobotDrive methods
        depositArmDrive.armLoop();

        // Real-time telemetry data to Driver Station
        telemetryManager.update("Front Left Motor Power", robot.frontLeftMotor.getPower());
        telemetryManager.update("Front Right Motor Power", robot.frontRightMotor.getPower());
        telemetryManager.update("Back Left Motor Power", robot.backLeftMotor.getPower());
        telemetryManager.update("Back Right Motor Power", robot.backRightMotor.getPower());
        // Update telemetry for the driver station


        //FTC DashBoard telemetry packet
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Front Left Motor Power", robot.frontLeftMotor.getPower());
        packet.put("Front Right Motor Power", robot.frontRightMotor.getPower());
        packet.put("Back Left Motor Power", robot.backLeftMotor.getPower());
        packet.put("Back Right Motor Power", robot.backRightMotor.getPower());
        packet.put("Lift Motor Left Position", robot.liftMotorLeft.getCurrentPosition());
        packet.put("Lift Motor Right Position", robot.liftMotorRight.getCurrentPosition());
        packet.put("IMU Heading",robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)); // Assuming there's a method to get IMU heading
        dashboard.sendTelemetryPacket(packet);
    }

    public void stop() {;
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.liftMotorLeft.setPower(0);
        robot.liftMotorRight.setPower(0);
        robot.IntakeServo.setPosition(1.0);
        telemetryManager.update("Status", "Robot stopped");
    }
}
