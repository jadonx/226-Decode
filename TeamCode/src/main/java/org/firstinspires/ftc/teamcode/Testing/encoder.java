package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AS5600Encoder;

@Config
@TeleOp(name = "Encoder testing")
public class encoder extends OpMode {

    AS5600Encoder encoderStuff;
    public void init(){
        encoderStuff = hardwareMap.get(AS5600Encoder.class, "encoder");

    }
    public void loop(){
        telemetry.addData("Angle", encoderStuff.getAngleDegrees());
        telemetry.update();
    }




}
