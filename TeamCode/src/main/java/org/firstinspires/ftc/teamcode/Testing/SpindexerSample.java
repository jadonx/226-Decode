package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.AS5600Encoder;

public class SpindexerSample {
    // Spindexer
    private CRServo spindexerServo;
    private AS5600Encoder spindexerEncoder;

    // Color Sensor
    private NormalizedColorSensor colorSensor;
    float[] hsv = new float[3];

    private ElapsedTime colorSensorTimer;

    // Color Sensor Logic
    public enum HolderStatus { NONE, GREEN, PURPLE }
    private HolderStatus[] holderStatuses = {HolderStatus.NONE, HolderStatus.NONE, HolderStatus.NONE};

    // Min/Max Angles For Each Spindexer Holder
    private int[][] holderAngles = {{96, 110}, {211, 227}, {334, 347}};
    // 3 Indexes For Each Holder
    // 2 Indexes: Sum Of Hue Data, Hue Data Count (We take average)
    private double[][] holderHueValues = new double[3][2];
    // Boolean values of whether each holder has been checked
    private boolean[] hueAvgCollected = new boolean[3];

    // Spindexer PID Values
    private double kP, kD;
    private double lastError;
    private ElapsedTime pidTimer;

    // Spindexer Launch Angles
    private int[] holderLaunchAngles = {172, 290, 53};
    private int holderLaunchIndex;
    private ElapsedTime launchTimer;

    public SpindexerSample(HardwareMap hardwareMap) {
        spindexerServo = hardwareMap.get(CRServo.class, Constants.HMServospinDexer);
        spindexerEncoder = hardwareMap.get(AS5600Encoder.class, Constants.HMSpindexerEncoder);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, Constants.HMSideColorSensor);
        colorSensorTimer = new ElapsedTime();

        kP = 0.005;
        kD = 0.0001;
        lastError = 0;

        pidTimer = new ElapsedTime();

        holderLaunchIndex = 0;
        launchTimer = new ElapsedTime();
    }

    /*
    SORTER SPIN CODE
     */
    public void goToAngle(double target) {
        double error = getAngleError(target, getAngle());

        double derivative = (error - lastError) / pidTimer.seconds();

        double output = (kP * error) + (kD * derivative);

        spindexerServo.setPower(output);

        lastError = error;
        pidTimer.reset();
    }

    public void constantSpin(double power) {
        spindexerServo.setPower(power);
    }

    public double getAngle() {
        return spindexerEncoder.getWrappedAngle();
    }

    public void updatePID(double kP, double kD) {
        this.kP = kP;
        this.kD = kD;
    }

    private double getAngleError(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;

        // Normalize error to the range (-180, 180]
        error = (error + 180) % 360 - 180;
        return error;
    }

    /*
    COLOR SENSOR CODE
     */
    public HolderStatus[] getHolderColors() {
        getHolderColorsHelper(0);
        getHolderColorsHelper(1);
        getHolderColorsHelper(2);

        return holderStatuses;
    }

    public void getHolderColorsHelper(int holderIndex) {
        double current = getAngle();

        int minAngle = holderAngles[holderIndex][0];
        int maxAngle = holderAngles[holderIndex][1];

        if (colorSensorTimer.milliseconds() > 10 && current > minAngle && current < maxAngle) {
            holderHueValues[holderIndex][0] += getHue();
            holderHueValues[holderIndex][1]++;
            colorSensorTimer.reset();

            hueAvgCollected[holderIndex] = false;
        }
        else if ((current < minAngle || current > maxAngle) && !hueAvgCollected[holderIndex]) {
            double avgHue = holderHueValues[holderIndex][0] / holderHueValues[holderIndex][1];

            if (avgHue > 150 && avgHue < 170) {
                holderStatuses[holderIndex] = HolderStatus.GREEN;
            }
            else if (avgHue > 230 && avgHue < 250) {
                holderStatuses[holderIndex] = HolderStatus.PURPLE;
            }
            else {
                holderStatuses[holderIndex] = HolderStatus.NONE;
            }

            holderHueValues[holderIndex][0] = 0;
            holderHueValues[holderIndex][1] = 0;
            hueAvgCollected[holderIndex] = true;
        }
    }

    public double getHue() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Convert RGB → HSV
        android.graphics.Color.RGBToHSV(
                (int) (colors.red * 255),
                (int) (colors.green * 255),
                (int) (colors.blue * 255),
                hsv
        );

        return hsv[0];
    }

    /*
    SORTER LAUNCH CODE
     */
    public void launchSpin() {
        if (launchTimer.seconds() > 1.5) {
            goToAngle(holderLaunchAngles[holderLaunchIndex%3]);
            holderLaunchIndex++;
            launchTimer.reset();
        }
    }
}