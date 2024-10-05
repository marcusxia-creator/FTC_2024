package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/* config
color_sensor I2C port 01
 */
public class ColSensorTest {
    private final HardwareMap hardwareMap;
    private ColorSensor _colorSensor;

    public ColSensorTest(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public void colSensor_ini(){
        _colorSensor = hardwareMap.get(ColorSensor.class,"color_sensor");
    }

    public double redColor(){
        return _colorSensor.red();
    }

}
