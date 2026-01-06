package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;

@TeleOp(name="Color Sensors Test")
public class ColorSensorsTest extends OpMode {
    RevColorSensorV3 colorSensorFront, colorSensorBack;

    @Override
    public void init() {
        colorSensorFront = hardwareMap.get(RevColorSensorV3.class, Constants.HMFrontColorSensor);
    }

    @Override
    public void loop() {

    }
}
