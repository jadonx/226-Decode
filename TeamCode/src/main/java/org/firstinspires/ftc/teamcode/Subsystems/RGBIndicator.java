package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class RGBIndicator {
    Servo light;
    private double lastValue = Double.NaN;

    public RGBIndicator(HardwareMap hardwareMap) {
        light = hardwareMap.get(Servo.class, Constants.HMServoLight);
    }

    public void setLightValue(double value) {
        if (value != lastValue) {
            light.setPosition(value);
            lastValue = value;
        }
    }
}
