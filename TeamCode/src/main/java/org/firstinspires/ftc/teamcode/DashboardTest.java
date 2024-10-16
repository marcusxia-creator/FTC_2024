package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name ="Dashboard Test ",group = "LinearOpMode")
public class DashboardTest extends LinearOpMode {

    private MotorEx motor;

    @Override
    public void runOpMode() {
        // Initialize motor
        motor = new MotorEx(hardwareMap, "FL_Motor", Motor.GoBILDA.RPM_435);

        // Set motor run mode
        motor.setRunMode(Motor.RunMode.RawPower);

        // Initialize FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Wait for the start command
        waitForStart();

        while (opModeIsActive()) {
            // Get motor power input from gamepad (right stick y-axis)
            // Invert the value because pushing the stick forward usually returns a negative value
            double motorPower = -gamepad2.right_stick_y;

            // Set motor power based on gamepad input
            motor.set(motorPower);

            // Get motor telemetry data (velocity and position)
            double motorVelocity = motor.getVelocity();
            int motorPosition = motor.getCurrentPosition();

            // Create a telemetry packet to send to FTC Dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Motor Power", motorPower);
            packet.put("Motor Velocity", motorVelocity);
            packet.put("Motor Position", motorPosition);

            // Send telemetry packet to the dashboard
            dashboard.sendTelemetryPacket(packet);

            // Display data in telemetry on the driver station (optional)
            telemetry.addData("Motor Power", motorPower);
            telemetry.addData("Motor Velocity", motorVelocity);
            telemetry.addData("Motor Position", motorPosition);
            telemetry.update();
        }
    }
}


