package org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Encoder.AS5600Encoder;

public class Spindexer {
    // Spindexer
    private CRServo spindexerServo;
    private AS5600Encoder spindexerEncoder;

    // Color Sensor
    private NormalizedColorSensor colorSensor;
    float[] hsv = new float[3];

    private ElapsedTime colorSensorTimer;

    // 3 Indexes For Each Holder
    // 2 Indexes: Sum Of Hue Data, Hue Data Count (We take average)
    private double[][] holderHueValues = new double[3][2];

    // Min/Max Angles For Each Spindexer Holder
    private int[][] holderAngles = {{35, 45}, {155, 165}, {275, 285}};

    public enum HolderStatus { NONE, GREEN, PURPLE }
    private HolderStatus[] holderStatuses = {HolderStatus.NONE, HolderStatus.NONE, HolderStatus.NONE};

    // COLOR SENSOR HOLDER TEST
    private double[] holderHueValue = new double[2];
    private HolderStatus holderStatus = HolderStatus.NONE;
    private double avgHue = 0;
    private boolean hueAvgCollected = false;

    // Spindexer PID Values
    private double kP, kD;
    private double lastError;
    private ElapsedTime pidTimer;

    public Spindexer(HardwareMap hardwareMap) {
        spindexerServo = hardwareMap.get(CRServo.class, "leftCRServo");
        spindexerEncoder = hardwareMap.get(AS5600Encoder.class, "spinEncoder");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensorTimer = new ElapsedTime();

        kP = 0.005;
        kD = 0.0001;
        lastError = 0;

        pidTimer = new ElapsedTime();
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
        return spindexerEncoder.getAngleDegrees();
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

    /*
    public HolderStatus[] getHolderColors() {
        double currentAngle = getAngle();

        getHolderColorsHelper(0, currentAngle);
        getHolderColorsHelper(1, currentAngle);
        getHolderColorsHelper(2, currentAngle);

        return holderStatus;
    }
     */

    // Green Hue: 150-180
    // Purple Hue: 150-240
    /*
    public void getHolderColorsHelper(int index, double currentAngle) {
        // Get angle range where color sensor can see holder
        int minAngle = holderAngles[index][0];
        int maxAngle = holderAngles[index][1];

        // If 10ms has passed (lag to help performance) and currentAngle is in between our desired angles
        if (colorSensorTimer.milliseconds() > 10 && currentAngle > minAngle && currentAngle < maxAngle) {
            // Add the detected hue to the hue sum
            holderHueValues[index][0] += getHue();
            // Add one to the amount of hues we've collected
            holderHueValues[index][1]++;
            // Reset timer
            colorSensorTimer.reset();
        }
        // Once our holder has passed the color sensor
        else if (currentAngle > maxAngle) {
            // Get the average hue detected in our holder
            double avgHue = holderHueValues[index][0] / holderHueValues[index][1];

            // Determine status of holder based on hue value
            if (avgHue > 160 && avgHue < 180) {
                holderStatus[index] = HolderStatus.GREEN;
            }
            else if (avgHue > 180 && avgHue < 240) {
                holderStatus[index] = HolderStatus.PURPLE;
            }
            else {
                holderStatus[index] = HolderStatus.NONE;
            }

            // Reset hue sum and hue count for next rotation
            holderHueValues[index][0] = 0;
            holderHueValues[index][1] = 0;
        }
    }
     */

    /*
    COLOR SENSOR CODE TEST METHODS (SINGLE HOLDER)
     */
    public double getHolderColor(int minAngle, int maxAngle) {
        double current = getAngle();

        if (colorSensorTimer.milliseconds() > 10 && current > minAngle && current < maxAngle) {
            holderHueValue[0] += getHue();
            holderHueValue[1]++;
            colorSensorTimer.reset();

            hueAvgCollected = false;
        }
        else if ((current < minAngle || current > maxAngle) && !hueAvgCollected) {
            avgHue = holderHueValue[0] / holderHueValue[1];

            if (avgHue > 150 && avgHue < 170) {
                holderStatus = HolderStatus.GREEN;
            }
            else if (avgHue > 230 && avgHue < 250) {
                holderStatus = HolderStatus.PURPLE;
            }
            else {
                holderStatus = HolderStatus.NONE;
            }

            holderHueValue[0] = 0;
            holderHueValue[1] = 0;
            hueAvgCollected = true;
        }

        return avgHue;
    }

    private double getHue() {
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
}
