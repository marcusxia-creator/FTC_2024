package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.util.Range;


@TeleOp (name="BasicTeleOps_ftclib", group = "linear OpMode")
public class BasicTeleOps extends OpMode {

    public RobotHardware robot;
    public GamepadEx gamepad;
    public MecanumDrive robotDrive;
    static final boolean FIELD_CENTRIC = false;
    public RevIMU imu;

    @Override
    public void init() {
        robot = new RobotHardware();
        robot.init(hardwareMap);
        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();
        MecanumDrive robotDrive = new MecanumDrive(robot.frontLeftMotor, robot.frontRightMotor, robot.backLeftMotor, robot.backRightMotor);
        gamepad = new GamepadEx(gamepad1);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        robotMecaDrive(FIELD_CENTRIC, robotDrive, gamepad);
        telemetry.addData("Status", "Robot running");
        telemetry.update();

    }

    private void robotMecaDrive(boolean FIELD_CENTRIC, MecanumDrive robotDrive, GamepadEx gamepad) {
        if (!FIELD_CENTRIC) {
            robotDrive.driveRobotCentric(
                    Range.clip(gamepad.getLeftX(), -1.0, 1.0),
                    Range.clip(gamepad.getLeftY(), -1.0, 1.0),
                    Range.clip(gamepad.getRightX(), -1.0, 1.0),
                    false
            );
        } else {
            robotDrive.driveFieldCentric(
                    Range.clip(gamepad.getLeftX(), -1.0, 1.0),
                    Range.clip(gamepad.getLeftY(), -1.0, 1.0),
                    Range.clip(gamepad.getRightX(), -1.0, 1.0),
                    imu.getRotation2d().getDegrees(),
                    false
            );
        }
    }

    public void stop() {
        robotDrive.stop();
        robot.frontLeftMotor.set(0);
        robot.frontRightMotor.set(0);
        robot.backLeftMotor.set(0);
        robot.backRightMotor.set(0);
        telemetry.addData("Status", "Robot stopped");
        telemetry.update();
    }
}

