package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class ColorSensor {
    private RevColorSensorV3 colorSensorFront;
    private RevColorSensorV3 colorSensorBack;

    private float[] hsv = new float[3];
    private double currentHue;
    private double currentDistance;
    private Spindexer.HolderStatus currentBall;
    private boolean hasBall;

    private ElapsedTime timer;
    private final int UPDATE_TIME = 150;

    public ColorSensor(HardwareMap hardwareMap) {
        colorSensorFront = hardwareMap.get(RevColorSensorV3.class, Constants.HMFrontColorSensor);
        // colorSensorBack = hardwareMap.get(RevColorSensorV3.class, Constants.HMBackColorSensor);

        currentBall = Spindexer.HolderStatus.NONE;

        timer = new ElapsedTime();
        timer.reset();
    }

    public void update() {
        if (timer.milliseconds() > UPDATE_TIME) {
            updateDistance();
            currentHue = hsv[0];

            if (currentDistance < 1.5) {
                hasBall = true;

//                updateColor();
//                if (currentHue > 215 && currentHue < 245) {
//                    currentBall = Spindexer.HolderStatus.PURPLE;
//                }
//                else if (currentHue > 145 && currentHue < 175) {
//                    currentBall = Spindexer.HolderStatus.GREEN;
//                }
            }
            else {
                hasBall = false;
                currentBall = Spindexer.HolderStatus.NONE;
            }

            timer.reset();
        }
    }

    public Spindexer.HolderStatus getCurrentBall() {
        return currentBall;
    }

    public boolean hasBall() {
        return hasBall;
    }

    private void updateDistance() {
        currentDistance = colorSensorFront.getDistance(DistanceUnit.INCH);
    }

    private void updateColor() {
        int r = colorSensorFront.red();
        int g = colorSensorFront.green();
        int b = colorSensorFront.blue();
        Color.RGBToHSV(r, g, b, hsv);
    }
}
