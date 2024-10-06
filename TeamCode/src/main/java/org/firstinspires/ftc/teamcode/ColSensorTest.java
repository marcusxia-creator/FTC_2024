package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

/* config
color_sensor I2C port 01
 */
public class ColSensorTest {
    private final RobotHardware robot;
    private final Telemetry telemetry;
    private int greenValue;
    private int redValue;
    private int blueValue;

    public ColSensorTest(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public int[] getColor() {
        if (robot.ColorSensor == null) {
            telemetry.addData("Error", "Color sensor not initialized");
            telemetry.update();
            return new int[]{0, 0, 0}; // return default values in case of error
        }
        redValue = robot.ColorSensor.red();
        greenValue = robot.ColorSensor.green();
        blueValue = robot.ColorSensor.blue();
        return new int[]{redValue, greenValue, blueValue};
    }

    /*
    public void displayColorTelemetry() {
        telemetry.addData("Red", redValue);
        telemetry.addData("Green", greenValue);
        telemetry.addData("Blue", blueValue);
        telemetry.update();
    }
     */
}
