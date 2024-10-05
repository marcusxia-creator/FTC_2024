package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

/* config
color_sensor I2C port 01
 */
public class ColSensorTest {
    private final HardwareMap hardwareMap;
    private ColorSensor _colorSensor;
    private double greenValue;
    private double redValue;
    private double blueValue;


    public ColSensorTest(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public void colSensor_ini(){
        _colorSensor = hardwareMap.get(ColorSensor.class,"color_sensor");
    }

    public double[] getColor(){
        redValue   =  _colorSensor.red();
        greenValue = _colorSensor.green();
        blueValue  =  _colorSensor.blue();
        //items.add((double) _colorSensor.alpha());
        return new double[]{redValue, greenValue, blueValue};
    }

}
