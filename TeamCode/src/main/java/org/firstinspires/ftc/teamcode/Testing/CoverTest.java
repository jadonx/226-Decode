package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

@Config
@TeleOp(name="CoverTest", group="Test")
public class CoverTest extends OpMode {
    Servo cover;
    public static double hoodPos;

    @Override
    public void init() {
        cover = hardwareMap.get(Servo.class, Constants.HMServobackSpin);
    }

    @Override
    public void loop() {
        cover.setPosition(hoodPos);
    }
}
