package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.SensorColor;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotHardware {
    public MotorEx frontLeftMotor;
    public MotorEx backLeftMotor;
    public MotorEx frontRightMotor;
    public MotorEx backRightMotor;
    public MotorEx leftodometry;
    public MotorEx rightodometry;
    public MotorEx centerodometry;

    public MotorEx verticalSlideMotor1;

    public ServoEx intakeServo;

    public SensorColor ColorSensor;

    public HardwareMap hardwareMap;

    public IMU imu;


    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap; // store the hardwareMap reference

        // set drivebase motors
        frontLeftMotor = new MotorEx(hardwareMap, "FL_Motor", Motor.GoBILDA.RPM_435);
        backLeftMotor = new MotorEx(hardwareMap, "BL_Motor", Motor.GoBILDA.RPM_435);
        frontRightMotor = new MotorEx(hardwareMap, "FR_Motor", Motor.GoBILDA.RPM_435);
        backRightMotor = new MotorEx(hardwareMap, "BR_Motor", Motor.GoBILDA.RPM_435);

        // set odometry
        leftodometry = new MotorEx(hardwareMap, "FL_Motor");// set odometry
        rightodometry = new  MotorEx(hardwareMap,"BL_Motor");// set odometry
        centerodometry = new MotorEx(hardwareMap,"FR_Motor");// set odometry

        //set vertical slide Motor
        verticalSlideMotor1 = new MotorEx(hardwareMap,"VS_Motor_1", Motor.GoBILDA.RPM_312);

        //set intake servo
        intakeServo = new SimpleServo(hardwareMap, "Intake_Servo",-150,150, AngleUnit.DEGREES);

        // reset encoder
        leftodometry.resetEncoder();
        rightodometry.resetEncoder();
        centerodometry.resetEncoder();

        //set motor mode and motor direction
        frontLeftMotor.setInverted(false);  // Reverse the left motor if needed
        backLeftMotor.setInverted(false);  // Reverse the left motor if needed
        frontLeftMotor.setRunMode(Motor.RunMode.RawPower); // set motor mode
        backLeftMotor.setRunMode(Motor.RunMode.RawPower); //set motor mode
        frontRightMotor.setRunMode(Motor.RunMode.RawPower); // set motor mode
        backRightMotor.setRunMode((Motor.RunMode.RawPower)); // set motor mode

        // set slide motor mode and motor direction
        verticalSlideMotor1.setRunMode(Motor.RunMode.PositionControl);
        verticalSlideMotor1.stopAndResetEncoder();

        // set robot motor power 0
        frontLeftMotor.set(0);
        frontRightMotor.set(0);
        backLeftMotor.set(0);
        backRightMotor.set(0);
        // reset encoders when stop
        resetDriveEncoders();

        // set color sensor
        ColorSensor = new SensorColor(hardwareMap, "Color_Sensor");

    }

    // Initialize IMU
    public void initIMU() {
        // get imu from hardwareMap
        //imu = hardwareMap.get(BNO055IMU.class, "Adafruit_IMU");
        // Initialize IMU parameter setup
        //BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        //imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // initialize IMU
        //imu.initialize(imuParameters);

        // set up REVImu
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD))
        );
    }

    // reset odometry encoder
    public void resetDriveEncoders(){
        leftodometry.stopAndResetEncoder();
        rightodometry.stopAndResetEncoder();
        centerodometry.stopAndResetEncoder();
    }
}