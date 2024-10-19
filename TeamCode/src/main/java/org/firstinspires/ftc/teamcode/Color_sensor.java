package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class Color_sensor {
    private RobotHardware robot;
    private int greenValue;
    private int redValue;
    private int blueValue;
    private int intensity;
    /*
    config
    color
     */
    public Color_sensor(RobotHardware robot){
        this.robot = robot;
    }

    public int[] getColor(){
        if (robot.Color_Sensor == null) {
            return new int[]{0, 0, 0}; // return default values in case of error
        }
        redValue = robot.Color_Sensor.red();
        greenValue = robot.Color_Sensor.green();
        blueValue = robot.Color_Sensor.blue();
        intensity = robot.Color_Sensor.alpha();
        return new int[]{redValue, greenValue, blueValue, intensity};
    }
}
