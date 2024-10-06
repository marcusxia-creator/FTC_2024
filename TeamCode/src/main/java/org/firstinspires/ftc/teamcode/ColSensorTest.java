package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

/* config
color_sensor I2C port 01
 */
public class ColSensorTest {
    private final RobotHardware robot;
    private double greenValue;
    private double redValue;
    private double blueValue;


    public ColSensorTest(RobotHardware robot){
        this.robot = robot;
    }

    public double[] getColor(){
        redValue   =  robot.ColorSensor.red();
        greenValue =  robot.ColorSensor.green();
        blueValue  =  robot.ColorSensor.blue();
        //items.add((double) _colorSensor.alpha());
        return new double[]{redValue, greenValue, blueValue};
    }

}
