package org.firstinspires.ftc.teamcode;

import static java.lang.String.format;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp (name="BasicTeleOps_FTCLib_MDrive", group = "linear OpMode")
public class BasicTeleOps extends OpMode {

    public RobotHardware robot;
    public GamepadEx gamepad;
    public MecanumDrive robotDrive;
    private final boolean FIELD_CENTRIC = false;

    //declare position and heading tracking variables
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;
    private int previousLeftEncoder = 0;
    private int previousRightEncoder = 0;
    private int previousCenterEncoder = 0;

    private static final double TICKS_PER_INCH = 1/0.07593;
    private static final double TRACK_WIDTH = 1/0.0001759;

    //declare telemetry parameters
    private Telemetry.Item status;
    private Telemetry.Item headings;
    private Telemetry.Item encoderHeadings;
    private Telemetry.Item encoderXPos;
    private Telemetry.Item encoderYPos;


    @Override
    public void init() {
        robot = new RobotHardware();
        robot.init(hardwareMap);
        robot.initIMU();
        robotDrive = new MecanumDrive(robot.frontLeftMotor, robot.frontRightMotor, robot.backLeftMotor, robot.backRightMotor);
        gamepad = new GamepadEx(gamepad2);
        telemetry.addData("Status", "Initialized");

        status = telemetry.addData("Status","running");
        headings = telemetry.addData("IMU Angle", format("%.2f", robot.imu.getRobotYawPitchRollAngles()));
        encoderHeadings = telemetry.addData("encoder_Headings ", format("%.2f", robotHeading));
        encoderXPos = telemetry.addData("encoder_X position ", 0);
        encoderYPos = telemetry.addData("encoder_Y position ", 0);
        telemetry.update();
    }

    @Override
    public void loop() {
        robotMecaDrive(!FIELD_CENTRIC, robotDrive, gamepad);
        updateTelemetry();

    }

    private void robotMecaDrive(boolean FIELD_CENTRIC, MecanumDrive robotDrive, GamepadEx gamepad) {
        final double powerFactor = 0.7;
        double  strafePower =  gamepad.getRightX();
        double  drivePower =   gamepad.getRightY();
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

    private void updateTelemetry(){
        headings = telemetry.addData("IMU Angle", format("%.2f", robot.imu.getRobotYawPitchRollAngles()));
        encoderXPos.setValue(String.format("%.2f", robotX));
        encoderYPos.setValue(String.format("%.2f", robotY));
        encoderHeadings.setValue(String.format("%.2f", robotHeading));

        telemetry.update();
    }

    private void updateOdometry(){
        //get the current encoder positions
        int currentLeftEncoder = robot.leftodometry.getCurrentPosition();
        int currentRightEncoder = robot.rightodometry.getCurrentPosition();
        int currentCenterEncoder = robot.centerodometry.getCurrentPosition();

        // Calculate the change in encoder values since the last update
        int deltaLeft = currentLeftEncoder - previousLeftEncoder;
        int deltaRight = currentRightEncoder - previousRightEncoder;
        int deltaCenter = currentCenterEncoder - previousCenterEncoder;

        // Store the current values for the next updates
        previousLeftEncoder = currentLeftEncoder;
        previousRightEncoder = currentRightEncoder;
        previousCenterEncoder = currentCenterEncoder;

        // Calculate the heading change ( in radians) using odometry (wheel base)
        double deltaHeading = (deltaRight - deltaLeft) / TRACK_WIDTH;

        // Update the robot's heading
        robotHeading += deltaHeading*(180/Math.PI);
        robotHeading = wrapAngle(robotHeading);  // Wrap heading between -π and π

        // Calculate the forward and lateral movement
        double deltaForward = (deltaLeft + deltaRight) / 2.0;
        double deltaForwardInches = deltaForward / TICKS_PER_INCH;
        double deltaLateralInches = deltaCenter / TICKS_PER_INCH;

        // Update the robot's X and Y position based on the heading and movements
        robotX += deltaForwardInches * Math.cos(robotHeading) - deltaLateralInches * Math.sin(robotHeading);
        robotY += deltaForwardInches * Math.sin(robotHeading) + deltaLateralInches * Math.cos(robotHeading);
    }
    // Function to wrap the angle between -π and π (radians)
    private double wrapAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}

