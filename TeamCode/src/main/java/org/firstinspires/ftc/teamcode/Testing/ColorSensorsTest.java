package org.firstinspires.ftc.teamcode.Testing;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;

@TeleOp(name="Color Sensors Test")
public class ColorSensorsTest extends OpMode {
    RevColorSensorV3 colorSensorFront, colorSensorBack;

    @Override
    public void init() {
        colorSensorFront = hardwareMap.get(RevColorSensorV3.class, Constants.HMFrontColorSensor);
        colorSensorBack = hardwareMap.get(RevColorSensorV3.class, Constants.HMBackColorSensor);
    }

    @Override
    public void loop() {
        telemetry.addData("Front HSV ", getHSVRev(colorSensorFront)[0]);
        telemetry.addData("Front Distance ", colorSensorFront.getDistance(DistanceUnit.INCH));
        telemetry.addData("Back HSV ", getHSVRev(colorSensorBack)[0]);
        telemetry.addData("Back Distance ", colorSensorBack.getDistance(DistanceUnit.INCH));

        telemetry.update();
    }

    public float[] getHSVRev(RevColorSensorV3 colorSensor) {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);

        return hsv;
    }
}
