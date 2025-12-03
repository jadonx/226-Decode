package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Encoder.AS5600Encoder;

public class Spindexer {
    // HARDWARE VARIABLES
    private CRServo spindexerServo;
    private AS5600Encoder spindexerEncoder;

    // PID VARIABLES
    private double kP = 0.005, kD = 0, kS = 0.12;
    private int slowingThreshold = 40;
    private double slowingMultiplier = 0.45;
    private int stoppingThreshold = 3;

    /*
    private double alpha = 0.1;
    private double filteredDerivative = 0;
    private double lastError = 0;
    private ElapsedTime pidTimer;
     */

    private int[] launchHolderAngles = {49, 181, 283};

    // COLOR SENSOR VARIABLES
    private NormalizedColorSensor colorSensor;
    private RevColorSensorV3 colorSensorV3;
    float[] hsv = new float[3];
    private ElapsedTime colorSensorTimer;

    public Spindexer(HardwareMap hardwareMap) {
        spindexerServo = hardwareMap.get(CRServo.class, Constants.HMServospinDexer);
        spindexerEncoder = hardwareMap.get(AS5600Encoder.class, Constants.HMSpindexerEncoder);

        // pidTimer = new ElapsedTime();

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, Constants.HMFrontColorSensor);
        colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, Constants.HMFrontColorSensor);
        colorSensorTimer = new ElapsedTime();
    }

    /*
    INTAKE CODE
     */
    public void runSpindexer() {
        spindexerServo.setPower(1);
    }

    public void runSpindexer(double power) {
        spindexerServo.setPower(power);
    }

    public void unjamSpindexer() {
        spindexerServo.setPower(-0.5);
    }

    public void stopSpindexer() {
        spindexerServo.setPower(0);
    }

    /*
    PID CODE
     */
    public void goToAngle(double target) {
        double error = getAngleError(target, getAngle());

        /* DERIVATIVE (NOT USED)
        double derivative = (error - lastError) / pidTimer.seconds();

        Derivative filtering to reduce oscillations
        filteredDerivative = alpha * derivative + (1-alpha) * filteredDerivative;

        double output = (kP * error) + (-kD * filteredDerivative);
         */

        double output = kP * error;

        // Feedforward to overcome static friction
        double ff = kS * Math.signum(error);

        if (Math.abs(error) < stoppingThreshold) {
            output = 0;
        }
        else if (Math.abs(error) < slowingThreshold) {
            output = (output + ff) * slowingMultiplier;
        }
        else {
            output = output + ff;
        }

        // FEEDFORWARD + RANGING/CLIPPING
        output  = Range.clip(output, -0.55, 0.55);

        spindexerServo.setPower(output);

        // lastError = error;
        // pidTimer.reset();
    }

    private double getAngleError(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;

        // Normalize error to the range (-180, 180]
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        return error;
    }

    public double getAngle() {
        return spindexerEncoder.getAngleDegrees();
    }

    public void updatePID(double kP, double kS, int slowingThreshold, double slowingMultiplier, int stoppingThreshold) {
        this.kP = kP;
        this.kS = kS;
        this.slowingThreshold = slowingThreshold;
        this.slowingMultiplier = slowingMultiplier;
        this.stoppingThreshold = stoppingThreshold;
    }

    // Returns the sequence of shooting angles starting from the closest (for shooting)
    public int[] getLaunchAngleSequence() {
        int current = (int) getAngle();

        int a = launchHolderAngles[0], b = launchHolderAngles[1], c = launchHolderAngles[2];

        int da = Math.abs(current - a);
        int db = Math.abs(current - b);
        int dc = Math.abs(current - c);

        if (da <= db && da <= dc) {
            return new int[] {a, b, c};
        }
        else if (db <= da && db <= dc) {
            return new int[] {b, c, a};
        }
        else {
            return new int[] {c, a, b};
        }
    }

    /*
    COLOR SENSOR CODE
     */
    public float[] getHue() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Convert RGB → HSV
        android.graphics.Color.RGBToHSV(
                (int) (colors.red * 255),
                (int) (colors.green * 255),
                (int) (colors.blue * 255),
                hsv
        );

        return hsv;
    }

    public float[] getHSVRev() {
        int r = colorSensorV3.red();
        int g = colorSensorV3.green();
        int b = colorSensorV3.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);

        return hsv;
    }

    public int getAlpha() {
        return colorSensorV3.alpha();
    }
}
