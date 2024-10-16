package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@Config
@TeleOp(name = "DualMotorSlideControl", group = "LinearOpMode")
public class DualMotorSlideControl extends LinearOpMode {

    // Motor names in Control Hub
    private MotorEx liftMotorLeft;
    private MotorEx liftMotorRight;
    private MotorGroup motorGroup;

    // PID coefficients (configurable through FTC Dashboard)
    public static double P = 0.1;
    public static double I = 0.0;
    public static double D = 0.0;

    // Positions and powers (configurable through FTC Dashboard)
    public static int upPosition = 1500;
    public static int downPosition = 100;
    public static double upPower = 0.5;
    public static double downPower = 0.4;
    public static int positionTolerance = 10;  // Tolerance in ticks (configurable)

    @Override
    public void runOpMode() {
        // Initialize motors
        liftMotorLeft = new MotorEx(hardwareMap, "VS_Motor_Left", MotorEx.GoBILDA.RPM_312);
        liftMotorRight = new MotorEx(hardwareMap, "VS_Motor_Right", MotorEx.GoBILDA.RPM_312);

        // Reverse one motor if necessary
        liftMotorRight.setInverted(true);

        // Group the two motors together
        motorGroup = new MotorGroup(liftMotorLeft, liftMotorRight);

        // Set motors to run with encoders
        motorGroup.setRunMode(MotorEx.RunMode.PositionControl);

        // Set PID coefficients
        motorGroup.setPositionCoefficients(P, I, D);

        // Set position tolerance
        motorGroup.setPositionTolerance(positionTolerance);

        // Initialize FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Wait for the start of the OpMode
        waitForStart();

        while (opModeIsActive()) {
            // Check for button presses
            if (gamepad1.x) {
                // Move slides up to the specified position
                motorGroup.setTargetPosition(upPosition);
                motorGroup.set(upPower); // Set power for moving up
            } else if (gamepad1.y) {
                // Move slides down to the specified position
                motorGroup.setTargetPosition(downPosition);
                motorGroup.set(downPower); // Set power for moving down
            }

            // Create telemetry packet for the dashboard
            TelemetryPacket packet = new TelemetryPacket();

            // Add motor telemetry data
            packet.put("Left Motor Position", liftMotorLeft.getCurrentPosition());
            packet.put("Right Motor Position", liftMotorRight.getCurrentPosition());
            packet.put("Target Position", motorGroup.getTargetPosition());
            packet.put("Motor Power", motorGroup.get());

            // Send telemetry packet to dashboard
            dashboard.sendTelemetryPacket(packet);

            // Show telemetry on driver station
            telemetry.addData("Left Motor Position", liftMotorLeft.getCurrentPosition());
            telemetry.addData("Right Motor Position", liftMotorRight.getCurrentPosition());
            telemetry.addData("Target Position", motorGroup.getTargetPosition());
            telemetry.update();
        }
    }
}
