package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

@Config
@TeleOp (name = "PIDFArm")
public class PIDF_Arm extends OpMode {
    private PIDController controller;

    public static  double p = 0, i = 0 , d = 0;
    public  static double f=0;

    public static int target =0;

    private final double ticks_in_degree = 537.7/360;

    private DcMotorEx arm_motor;
    private ColorSensor colorSensor;

    private int redValue;
    private int greenValue;
    private int blueValue;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "HS_Motor");
        colorSensor = hardwareMap.get(ColorSensor.class, "Color_Sensor");
    }

    @Override
    public void loop(){
        controller.setPID(p,i,d);
        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;
        arm_motor.setPower(power);

        telemetry.addData("pos ", armPos);
        telemetry.addData("target int ", target);
        telemetry.addData("Color Sensor ", colorSensor.red());

        telemetry.update();



    }

}
