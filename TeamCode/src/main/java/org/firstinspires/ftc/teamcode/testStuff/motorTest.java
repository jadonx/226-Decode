package org.firstinspires.ftc.teamcode.testStuff;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;


import java.util.List;

@Config
@TeleOp(name = "Motor testing")

public class motorTest extends OpMode {
    //DcMotorEx motor1, motor2;
    NormalizedColorSensor colorSensor;
    float[] hsvValues = new float[3];


    public void init(){
        //motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        //motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");


    }
    public void loop(){
        //motor1.setVelocity(gamepad1.left_stick_y * 2800);
        //motor2.setVelocity(gamepad1.right_stick_y *2800);

        NormalizedRGBA colors = colorSensor.getNormalizedColors();



        // Then, add the normalized color values to telemetry
        telemetry.addData("Red", colors.red*255);
        telemetry.addData("Green", colors.green*255);
        telemetry.addData("Blue", colors.blue*255);
        telemetry.addData("Alpha", colors.alpha);

        float[] hsv = new float[3];
        Color.colorToHSV(colors.toColor(), hsv);
        telemetry.addData("Hue", hsv[0]);
        telemetry.addData("Saturation", hsv[1]);
        telemetry.addData("Light", hsv[2]);
    }

}

