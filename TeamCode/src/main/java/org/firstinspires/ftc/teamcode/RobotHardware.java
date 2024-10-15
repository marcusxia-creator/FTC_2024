package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

public class RobotHardware {
    public DcMotorEx frontLeftMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;
    //public MotorEx leftodometry; //FTClib declear
    //public MotorEx rightodometry;
    //public MotorEx centerodometry;
    public DcMotorEx liftMotorLeft;
    public DcMotorEx liftMotorRight;
    public Servo IntakeServo;
    public Servo IntakeArmServo;
    public SensorColor Color_Sensor;
    
    public IMU imu;
    public HardwareMap hardwareMap;

    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap; // store the hardwareMap reference
    //set Motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FL_Motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BL_Motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FR_Motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BR_Motor");
        liftMotorLeft = hardwareMap.get(DcMotorEx.class,"VS_Motor_Left");
        liftMotorRight = hardwareMap.get(DcMotorEx.class, "VS_motor_Right");

    //set intake gribber servo
        //intakeServo = new SimpleServo(hardwareMap, "IntakeArm_Servo",90,180, AngleUnit.DEGREES);// FTClib type map
        IntakeServo = hardwareMap.get(Servo.class, "Intake_Servo");
        IntakeArmServo = hardwareMap.get(Servo.class, "IntakeArm_Servo");

    //set color sensor
        Color_Sensor = hardwareMap.get(SensorColor.class,"Color_Sensor");

    // set odometry
        //leftodometry = new MotorEx(hardwareMap, "FL_Motor");// set odometry
        //rightodometry = new MotorEx(hardwareMap, "BL_Motor");// set odometry
        //centerodometry = new MotorEx(hardwareMap, "FR_Motor");// set odometry
        // reset encoder
        //leftodometry.resetEncoder();
        //rightodometry.resetEncoder();
        //centerodometry.resetEncoder();

    //set motor mode and motor direction
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);  // Reverse the left motor if needed
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);  // Reverse the left motor if needed
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // set motor mode
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //set motor mode
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // set motor mode
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // set motor mode

    //set to RUN_TO_POSITION for vertical slide motor
        liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // set robot motor power 0
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }// End of init
    
    // Initialize IMU
    public void initIMU() {
        /* get imu from hardwareMap for external IMU
        imu = hardwareMap.get(BNO055IMU.class, "Adafruit_IMU");
        Initialize IMU parameter setup
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // initialize IMU
        imu.initialize(imuParameters);
        */
    // set up REVimu
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD))
        );
    }
    // reset encoders when stop
    /*
    public void resetDriveEncoders() {
        leftodometry.stopAndResetEncoder();
        rightodometry.stopAndResetEncoder();
        centerodometry.stopAndResetEncoder();
    }*/
}
