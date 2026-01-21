package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.HMServoLight;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Light {
    Servo light;

    public static double lightValue = 0.33;

    public Light(HardwareMap hardwaremap){
        light = hardwaremap.get(Servo.class, HMServoLight);
    }

    public void turnOn1(){
        light.setPosition(lightValue);
    }

    public void turnOn2(){
        light.setPosition(0.66);
    }

    public void turnOn3(){
        light.setPosition(1);
    }

    public void turnOff(){
        light.setPosition(0);
    }
    //s
}
