package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {
    public MotorEx frontLeftMotor;
    public MotorEx backLeftMotor;
    public MotorEx frontRightMotor;
    public MotorEx backRightMotor;
    public MotorEx leftodometry;
    public MotorEx rightodometry;
    public MotorEx centerodometry;

    public HardwareMap hardwareMap;


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
        // reset encoder
        leftodometry.resetEncoder();
        rightodometry.resetEncoder();
        centerodometry.resetEncoder();

        //set motor mode and motor direction
        frontLeftMotor.setInverted(true);  // Reverse the left motor if needed
        backLeftMotor.setInverted(true);  // Reverse the left motor if needed
        frontLeftMotor.setRunMode(Motor.RunMode.RawPower); // set motor mode
        backLeftMotor.setRunMode(Motor.RunMode.RawPower); //set motor mode
        frontRightMotor.setRunMode(Motor.RunMode.RawPower); // set motor mode
        backRightMotor.setRunMode((Motor.RunMode.RawPower)); // set motor mode

        // set robot motor power 0
        frontLeftMotor.set(0);
        frontRightMotor.set(0);
        backLeftMotor.set(0);
        backRightMotor.set(0);
        // reset encoders when stop
        resetDriveEncoders();

    }

    // reset odometry encoder
    public void resetDriveEncoders(){
        leftodometry.stopAndResetEncoder();
        rightodometry.stopAndResetEncoder();
        centerodometry.stopAndResetEncoder();
    }
}