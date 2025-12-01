package org.firstinspires.ftc.teamcode.Testing;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;

@TeleOp(name = "Rev Color Sensor Test")
public class RevColorSensorTest extends OpMode {
    RevColorSensorV3 colorSensor;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, Constants.HMFrontColorSensor);
    }

    @Override
    public void loop() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        int alpha = colorSensor.alpha();
        double proximity = colorSensor.getDistance(DistanceUnit.INCH);

        int brightness = r + g + b;

        float hsv[] = new float[3];
        Color.RGBToHSV(r, g, b, hsv);

        telemetry.addData("R ", r);
        telemetry.addData("G ", g);
        telemetry.addData("B ", b);
        telemetry.addData("brightness ", brightness);
        telemetry.addData("alpha ", alpha);
        telemetry.addData("proximity ", proximity);
        telemetry.addData("hue ", hsv[0]);

        telemetry.update();
    }
}
