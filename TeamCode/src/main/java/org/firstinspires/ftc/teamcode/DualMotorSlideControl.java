package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    public static int upPosition = 1000;
    public static int downPosition = 500;
    public static double upPower = 0.1;
    public static double downPower = 0.1;
    public static int positionTolerance = 5;  // Tolerance in ticks (configurable)

    @Override
    public void runOpMode() {
        // Initialize motors
        liftMotorLeft = new MotorEx(hardwareMap, "FL_Motor", Motor.GoBILDA.RPM_435);
        liftMotorRight = new MotorEx(hardwareMap, "BL_Motor", Motor.GoBILDA.RPM_435);

        // Reverse one motor if necessary
        liftMotorLeft.setInverted(true);

        liftMotorLeft.resetEncoder();
        liftMotorRight.resetEncoder();
        liftMotorLeft.stopAndResetEncoder();
        liftMotorRight.stopAndResetEncoder();

        // Initialize FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Wait for the start of the OpMode
        waitForStart();

        while (opModeIsActive()) {
            // Check for button presses
            if (gamepad1.x) {
                // Move slides up to the specified position
                liftMotorLeft.setTargetPosition(upPosition);
                liftMotorLeft.set(0);
                while (!liftMotorLeft.atTargetPosition()){
                    liftMotorLeft.set(upPower); // Set power for moving up
                }
                liftMotorLeft.stopMotor();
            } else if (gamepad1.y) {
                // Move slides down to the specified position
                liftMotorLeft.setTargetPosition(upPosition*-1);
                liftMotorLeft.set(0);
                while (!liftMotorLeft.atTargetPosition()){
                    liftMotorLeft.set(upPower); // Set power for moving up
                }
                liftMotorLeft.stopMotor();
            }

            // Create telemetry packet for the dashboard
            TelemetryPacket packet = new TelemetryPacket();

            // Add motor telemetry data
            packet.put("Left Motor Position", liftMotorLeft.getCurrentPosition());
            packet.put("Right Motor Position", liftMotorRight.getCurrentPosition());
            packet.put("Motor Power", liftMotorLeft.get());

            // Send telemetry packet to dashboard
            dashboard.sendTelemetryPacket(packet);

            // Show telemetry on driver station
            telemetry.addData("Left Motor Position", liftMotorLeft.getCurrentPosition());
            telemetry.addData("Right Motor Position", liftMotorRight.getCurrentPosition());
            telemetry.addData("Power", liftMotorLeft.get());
            telemetry.update();
        }
    }
}
