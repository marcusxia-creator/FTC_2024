package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.UtilityOctoQuadConfigMenu;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class BasicTeleOps extends OpMode {

    public RobotHardware robot;
    public GamepadEx gamepad;
    public Telemetry telemetry;
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
        new GamepadEx(gamepad1);
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
                    gamepad.getLeftX(),
                    gamepad.getLeftY(),
                    gamepad.getRightX(),
                    false
            );
        } else {
            robotDrive.driveFieldCentric(
                    gamepad.getLeftX(),
                    gamepad.getLeftY(),
                    gamepad.getRightX(),
                    imu.getRotation2d().getDegrees(),
                    false
            );
        }
    }

    public void stop() {
        robotDrive.stop();
        telemetry.addData("Status", "Robot stopped");
        telemetry.update();
    }
}

