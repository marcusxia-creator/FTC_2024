package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp (name="BasicTeleOps_Testbot", group = "OpMode")
public class BasicTeleOps extends OpMode {

    public RobotHardware robot;
    public GamepadEx gamepadControl2;
    public RobotMecaDrive robotDrive;
    //public ColSensorTest ColorSensor; // declare color sensor
    public VslideControl vslideControl;

    /*
    //declare position and heading tracking variables
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.00;
    private int previousLeftEncoder = 0;
    private int previousRightEncoder = 0;
    private int previousCenterEncoder = 0;
    */

    private static final double TICKS_PER_INCH = 1/0.07593;
    private static final double TRACK_WIDTH = 1/0.0001759;

    //declare telemetry parameters
    private Telemetry.Item status;
    private Telemetry.Item headings;
    private Telemetry.Item encoderHeadings;
    private Telemetry.Item encoderXPos;
    private Telemetry.Item encoderYPos;
    private Telemetry.Item _color_red;
    private Telemetry.Item _color_blue;
    private Telemetry.Item _color_green;
    //Telemetry item
    private Telemetry.Item leftFrontMotorPower;
    private Telemetry.Item rightFrontMotorPower;
    private Telemetry.Item leftBackMotorPower;
    private Telemetry.Item rightBackMotorPower;


    @Override
    public void init() {
        // Initialize robot and robot imu
        robot = new RobotHardware(); // config robot
        robot.init(hardwareMap); // read in hardwareMap into robot for config hardwares
        robot.initIMU(); // initialize control hub IMU

        // initial control gamepad
        gamepadControl2 = new GamepadEx(gamepad2);

        // initial Mecanmum drive
        robotDrive = new RobotMecaDrive(robot, gamepadControl2,0.5);
        robotDrive.MecanDriveInitial();

        //initial Vertical Slides
        vslideControl = new VslideControl(robot, gamepadControl2, telemetry);
        vslideControl.VslideInitial();

        //initiate colorsensor
        //ColorSensor = new ColSensorTest(robot, telemetry);

        // telemetry
        status  = telemetry.addData("Status", "Initialized");
        headings = telemetry.addData("IMU Angle", "%.2f", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        leftFrontMotorPower = telemetry.addData("leftFrontMotorPower ", 0.0);
        rightFrontMotorPower = telemetry.addData("rightFrontMotorPower ", 0.0);
        leftBackMotorPower = telemetry.addData("lefBackMotorPower ", 0.0);
        rightBackMotorPower = telemetry.addData("rightBackMotorPower ", 0.0);
        /*
        encoderHeadings = telemetry.addData("encoder_Headings ", "%.2f", robotHeading);
        encoderXPos = telemetry.addData("encoder_X position ", 0);
        encoderYPos = telemetry.addData("encoder_Y position ", 0);
        _color_red = telemetry.addData("Red",valueOf(0.0));
        _color_blue = telemetry.addData("Blue",valueOf(0.0));
        _color_green = telemetry.addData("Green",valueOf(0.0));
        */
        telemetry.update();
    }

    @Override
    public void loop() {
        robotDrive.MecanDriveLoop();
        vslideControl.VslideRun();
        updateTelemetry();// show color sensor signal as well

    }

    public void stop() {
        robotDrive.stop();
        robot.frontLeftMotor.set(0);
        robot.frontRightMotor.set(0);
        robot.backLeftMotor.set(0);
        robot.backRightMotor.set(0);
        robot.verticalSlideMotorLeft.set(0);
        robot.verticalSlideMotorRight.set(0);
        //robot.intakeServo.setPosition(0.2);
        telemetry.addData("Status", "Robot stopped");
        telemetry.update();
    }

    private void updateTelemetry(){
        status.setValue("Status","running");
        //Update IMU Angle Correctly using yaw()
        headings.setValue(String.format("%.2f", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
        //encoderXPos.setValue(String.format("%.2f", robotX));
        //encoderYPos.setValue(String.format("%.2f", robotY));
        //encoderHeadings.setValue(String.format("%.2f", robotHeading));

        /*
        //Update color sensor values correctly
        _color_red.setValue(ColorSensor.getColor()[0]);
        _color_blue.setValue(ColorSensor.getColor()[2]);
        _color_green.setValue(ColorSensor.getColor()[1]);
        */
        leftFrontMotorPower.setValue(String.format("%.2f", robot.frontLeftMotor.getVelocity()));
        rightFrontMotorPower.setValue(String.format("%.2f", robot.frontRightMotor.getVelocity()));
        leftBackMotorPower.setValue(String.format("%.2f", robot.backLeftMotor.getVelocity()));
        rightBackMotorPower.setValue(String.format("%.2f", robot.backRightMotor.getVelocity()));
        telemetry.update();
    }

    /*
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
    */
}

