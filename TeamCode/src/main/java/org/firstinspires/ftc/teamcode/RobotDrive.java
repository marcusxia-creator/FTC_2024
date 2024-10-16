package org.firstinspires.ftc.teamcode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.BACK;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.START;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotDrive {

    private final GamepadEx gamepad;
    private final RobotHardware robot;
    private final double powerFactor;
    private ControlMode controlMode = ControlMode.FIELD_CENTRIC;
    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private final TelemetryManager telemetryManager;


    private boolean startPressed = false;
    private boolean backPressed = false;
    //declare Telemetry items
    private Telemetry.Item status;
    private Telemetry.Item imuAngle;
    private Telemetry.Item leftEncoder;
    private Telemetry.Item rightEncoder;
    private Telemetry.Item drivePower;
    private Telemetry.Item strafePower;
    private Telemetry.Item rotatePower;
    private Telemetry.Item motorVelocities;
    private Telemetry.Item encoderCounts;
    private Telemetry.Item heading;
    private Telemetry.Item controlModeItem;
    

    public RobotDrive(RobotHardware robot, GamepadEx gamepad, TelemetryManager telemetryManager, double powerFactor) {
        this.robot = robot;
        this.gamepad = gamepad;
        this.telemetryManager = telemetryManager;
        this.powerFactor = powerFactor;
    }

    public void init() {
        // Initialize IMU from RobotHardware
        robot.initIMU();
        telemetry.addData("Status", "Initializing...");

        // Initialize telemetry items for dynamic updates
        status = telemetry.addData("Status", "Run Time: ");
        imuAngle = telemetry.addData("IMU Angle", "Initializing...");
        leftEncoder = telemetry.addData("Left Encoder", 0);
        rightEncoder = telemetry.addData("Right Encoder", 0);
        drivePower = telemetry.addData("Drive Power", 0);
        strafePower = telemetry.addData("Strafe Power", 0);
        rotatePower = telemetry.addData("Rotate Power", 0);
        motorVelocities = telemetry.addData("Motor Velocities", "");
        encoderCounts = telemetry.addData("Encoder Counts", "");
        heading = telemetry.addData("Heading", 0);
        controlModeItem = telemetry.addData("Control Mode", controlMode.toString());
        telemetry.update();
    }

    @SuppressLint("DefaultLocale")
    public void driveLoop() {
        // Toggle control mode
        if (gamepad.getButton(START) && !startPressed) {
            toggleControlMode();
            debounceTimer.reset();
            startPressed = true;
        } else if (!gamepad.getButton(START)) {
            startPressed = false;
        }

        // Reset IMU heading using button back and reset odometry
        if (gamepad.getButton(BACK) && !backPressed) {
            robot.initIMU();
            //robot.resetDriveEncoders();
            debounceTimer.reset();
            backPressed = true;
        } else if (!gamepad.getButton(BACK)) {
            backPressed = false;
        }

        // Set gamepad joystick power
        double drive = -gamepad.getRightY();
        double strafe = gamepad.getRightX();
        double rotate = gamepad.getLeftX();

        // Get robot's current heading
        double currentHeading = getRobotHeading();

        // Mecanum drive calculations
        setMecanumDrivePower(drive, strafe, rotate, currentHeading);

        // Update telemetry with the latest data
        updateTelemetry(drive, strafe, rotate, currentHeading);
    }// end of driveloop

   private void updateTelemetry(double drive, double strafe, double rotate, double currentHeading) {
        telemetryManager.update("Run Time", debounceTimer.seconds());
        telemetryManager.update("IMU Angle", currentHeading);
        telemetryManager.update("Drive Power", drive);
        telemetryManager.update("Strafe Power", strafe);
        telemetryManager.update("Rotate Power", rotate);
        telemetryManager.update("Motor Velocities", String.format("LF:%g, RF:%g, LB:%g, RB:%g",
                getVelocity()[0], getVelocity()[1], getVelocity()[2], getVelocity()[3]));
        telemetryManager.update("Heading", currentHeading);
        telemetryManager.update("Control Mode", controlMode.toString());
    }//end of Telemetry

    private double getRobotHeading() {
        // Get the robot's heading from IMU
        // double heading = robot.imu().firstAngle;
        double heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        while (heading > 180.0) {
            heading -= 360.0;
        }
        while (heading < -180.0) {
            heading += 360.0;
        }
        return -heading;
    }

    private void toggleControlMode() {
        if (controlMode == ControlMode.FIELD_CENTRIC) {
            controlMode = ControlMode.ROBOT_CENTRIC;
        } else {
            controlMode = ControlMode.FIELD_CENTRIC;
        }
    }

    private void setMecanumDrivePower(double drive, double strafe, double rotate, double currentHeading) {
        // Determine the drive mode
        if (controlMode == ControlMode.FIELD_CENTRIC) {
            // Adjust for field-centric control using the gyro angle
            double headingRad = Math.toRadians(currentHeading);
            double temp = drive * Math.cos(headingRad) + strafe * Math.sin(headingRad);
            strafe = -drive * Math.sin(headingRad) + strafe * Math.cos(headingRad);
            drive = temp;
        }

        // Mecanum wheel drive formula
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;

        // Constrain the power within +-1.0
        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backRightPower), Math.abs(backLeftPower))
        );

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
            backLeftPower /= maxPower;
        }

        // Set motor powers
        robot.frontLeftMotor.setPower(Range.clip(frontLeftPower * powerFactor, -1.0, 1.0));
        robot.frontRightMotor.setPower(Range.clip(frontRightPower * powerFactor, -1.0, 1.0));
        robot.backLeftMotor.setPower(Range.clip(backLeftPower * powerFactor, -1.0, 1.0));
        robot.backRightMotor.setPower(Range.clip(backRightPower * powerFactor, -1.0, 1.0));

    }

    // Method to get left encoder count
    /*
    public int [] getEncoderCounts() {
        int[] counts = new int[3];
        counts[0] = robot.leftodometry.getCurrentPosition();
        counts[1] = robot.rightodometry.getCurrentPosition();
        counts[2] = robot.centerodometry.getCurrentPosition();
        return counts;
    }
    */
    public double[] getVelocity() {
        double[] velocities = new double[4];
        velocities[0] = robot.frontLeftMotor.getVelocity();
        velocities[1] = robot.frontRightMotor.getVelocity();
        velocities[2] = robot.backLeftMotor.getVelocity();
        velocities[3] = robot.backRightMotor.getVelocity();
        return velocities;
    }

    public enum ControlMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }
}
