package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
    public Color_sensor colorSensor;
    private FtcDashboard dashboard;
    private TelemetryManager telemetryManager;

    public static double powerFactor = 0.5;
    public static double dump_Idle = 0.9;
    public static double dump_Deposit = 0.1;
    public static double dropTime = 1.5;
    public static double retractTime = 2.8;
    public static double intake_Idle = 0.3;
    public static double intake_Dump = 0.0;
    //slides position
    public static int downLiftpos = 100;
    public static int upLiftmid = 1200;
    public static int upLiftpos = 2500;
    //slides power
    public static double upLiftPower = 0.7;
    public static double downLiftPower = 0.5;

    
    @Override
    public void init() {
        //telemetryManager = new TelemetryManager(telemetry);
        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());
        telemetryManager = new TelemetryManager(telemetry);
        robot = new RobotHardware();
        robot.init(hardwareMap); // Initialize hardware in RobotHardware
        gamepadCo1 = new GamepadEx(gamepad2);
        robotDrive = new RobotDrive(robot, gamepadCo1, telemetryManager,powerFactor); // Pass robot instance to RobotDrive
        robotDrive.init(); // Initialize RobotDrive
        depositArmDrive = new FiniteMachineStateArm(robot, gamepadCo1, telemetryManager, dump_Idle, dump_Deposit, dropTime, retractTime, intake_Idle, intake_Dump, downLiftpos, upLiftmid,upLiftpos, upLiftPower, downLiftPower); // Pass parameters as needed);
        depositArmDrive.init();
        colorSensor = new Color_sensor(robot);
        //dashboard = FtcDashboard.getInstance();
        telemetry.addLine("-------------------");
        telemetryManager.update("Status"," initialized Motors and Encoder and IMU and Arm Control");
        telemetry.addLine("-------------------");
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
        telemetry.addData("Motor Left Position", robot.liftMotorLeft.getCurrentPosition());
        telemetry.addData("Lift Motor Right Position", robot.liftMotorRight.getCurrentPosition());
        telemetryManager.update("Color Sensor red", colorSensor.getColor()[0]);
        telemetryManager.update("Color Sensor red", colorSensor.getColor()[0]);
        telemetryManager.update("Color Sensor red", colorSensor.getColor()[0]);
        telemetryManager.update("Lift State", depositArmDrive.State().toString());
        telemetryManager.update("Servo Intake position", robot.IntakeServo.getPosition());
        telemetryManager.update("Servo Intake Arm position", robot.IntakeArmServo.getPosition());
        telemetryManager.update("lift motor TP", robot.liftMotorLeft.getTargetPosition());
        telemetryManager.update("Right motor TP", robot.liftMotorRight.getTargetPosition());

        // Update telemetry for the driver station
        //FTC DashBoard telemetry packet
        /*
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Front Left Motor Power", robot.frontLeftMotor.getPower());
        packet.put("Front Right Motor Power", robot.frontRightMotor.getPower());
        packet.put("Back Left Motor Power", robot.backLeftMotor.getPower());
        packet.put("Back Right Motor Power", robot.backRightMotor.getPower());
        packet.put("Lift Motor Left Position", robot.liftMotorLeft.getCurrentPosition());
        packet.put("Lift Motor Right Position", robot.liftMotorRight.getCurrentPosition());
        packet.put("IMU Heading",robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)); // Assuming there's a method to get IMU heading
        dashboard.sendTelemetryPacket(packet);
        */
    }

    public void stop() {
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
