package org.firstinspires.ftc.teamcode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.START;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotMecaDrive {
    /*
    config gamepad2 for movement control
    right Y - forward
    right x - strafe
    left x - rotate

    toggle button for filed centric and robot centric
    STATE button
     */
    private final RobotHardware robot;
    private final GamepadEx gamepad;
    private ControlMode controlMode = ControlMode.FIELD_CENTRIC;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private MecanumDrive drive;

    // set button for robotic drive condition
    private boolean startPressed = false;

    public RobotMecaDrive(RobotHardware robot,GamepadEx gamepad){
        this.robot = robot;
        this.gamepad = gamepad;
    }

    public void MecanDriveInitial(){
        MecanumDrive drive = new MecanumDrive(robot.frontLeftMotor, robot.frontRightMotor, robot.backLeftMotor, robot.backRightMotor);
    }

    public void MecanDriveLoop() {
        final double powerFactor = 0.7;
        double strafePower = gamepad.getRightX();
        double drivePower = gamepad.getRightY();
        double rotatePower = gamepad.getLeftX();
        // Toggle control mode
        if (gamepad.getButton(START) && !startPressed) {
            toggleControlMode();
            debounceTimer.reset();
            startPressed = true;
        } else if (!gamepad.getButton(START)) {
            startPressed = false;
        }

        if (controlMode != ControlMode.FIELD_CENTRIC) {
            drive.driveRobotCentric(
                    Range.clip(strafePower * powerFactor, -1.0, 1.0),
                    Range.clip(drivePower * powerFactor, -1.0, 1.0),
                    Range.clip(rotatePower * powerFactor, -1.0, 1.0),
                    false
            );
        } else {
            drive.driveFieldCentric(
                    Range.clip(strafePower * powerFactor, -1.0, 1.0),
                    Range.clip(drivePower * powerFactor, -1.0, 1.0),
                    Range.clip(rotatePower * powerFactor, -1.0, 1.0),
                    robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),
                    false
            );
        }
    }
        // set control mode - field centric or robotic centric
    private void toggleControlMode () {
        if (controlMode == ControlMode.FIELD_CENTRIC) {
            controlMode = ControlMode.ROBOT_CENTRIC;
        } else {
            controlMode = ControlMode.FIELD_CENTRIC;
        }
    }

    public void stop() {
        robot.frontLeftMotor.set(0);
        robot.frontRightMotor.set(0);
        robot.backLeftMotor.set(0);
        robot.backRightMotor.set(0);
    }

    // set control mode states
    public enum ControlMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }
}
