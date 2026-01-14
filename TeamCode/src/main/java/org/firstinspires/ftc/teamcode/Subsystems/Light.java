package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Light {
    Servo light;

    public Light(HardwareMap hardwareMap) {
        light = hardwareMap.get(Servo.class, Constants.HMServoLight);
    }

    public void setLightValue(double value) {
        light.setPosition(value);
    }
}
