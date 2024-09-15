package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp (name="BasicTeleOps_FTCLib_MDrive", group = "linear OpMode")
public class BasicTeleOps extends OpMode {

    public RobotHardware robot;
    public GamepadEx gamepad;
    public MecanumDrive robotDrive;
    private final boolean FIELD_CENTRIC = false;

    @Override
    public void init() {
        robot = new RobotHardware();
        robot.init(hardwareMap);
        robot.initIMU();
        robotDrive = new MecanumDrive(robot.frontLeftMotor, robot.frontRightMotor, robot.backLeftMotor, robot.backRightMotor);
        gamepad = new GamepadEx(gamepad2);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        robotMecaDrive(!FIELD_CENTRIC, robotDrive, gamepad);
        telemetry.addData("Status", "Robot running");
        telemetry.update();

    }

    private void robotMecaDrive(boolean FIELD_CENTRIC, MecanumDrive robotDrive, GamepadEx gamepad) {
        final double powerFactor = 0.7;
        double  strafePower =  gamepad.getRightX();
        double  drivePower =  -gamepad.getRightY();
        double  rotatePower =  gamepad.getLeftX();

        if (!FIELD_CENTRIC) {
            robotDrive.driveRobotCentric(
                    Range.clip(strafePower*powerFactor, -1.0, 1.0),
                    Range.clip(drivePower*powerFactor  , -1.0, 1.0),
                    Range.clip(rotatePower*powerFactor, -1.0, 1.0),
                    false
            );
        } else {
            robotDrive.driveFieldCentric(
                    Range.clip(strafePower*powerFactor, -1.0, 1.0),
                    Range.clip(drivePower*powerFactor, -1.0, 1.0),
                    Range.clip(rotatePower*powerFactor, -1.0, 1.0),
                    robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),
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

