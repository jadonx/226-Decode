package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class Spindexer {
    // HARDWARE VARIABLES
    private CRServo spindexerServo;
    private AS5600Encoder spindexerEncoder;

    // PID VARIABLES
    private double kP = 0.005, kS = 0.04;

    // SPINDEXER ANGLE VALUES AND HOLDER STATUSES
    private int[] launchHolderAngles = {103, 228, 348};
    public enum HolderStatus { NONE, GREEN, PURPLE }
    public HolderStatus[] holderStatuses = {HolderStatus.NONE, HolderStatus.NONE, HolderStatus.NONE};

    // COLOR SENSOR VARIABLES
    private RevColorSensorV3 colorSensorV3;
    float[] hsv = new float[3];
    private ElapsedTime colorSensorTimer;

    public Spindexer(HardwareMap hardwareMap) {
        spindexerServo = hardwareMap.get(CRServo.class, Constants.HMServospinDexer);
        spindexerServo.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexerEncoder = hardwareMap.get(AS5600Encoder.class, Constants.HMSpindexerEncoder);

        colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, Constants.HMFrontColorSensor);
        colorSensorTimer = new ElapsedTime();
    }

    //toString method
    public String printPattern() {
        return "Spindexer{" +
                "holderStatuses=" + holderStatuses[0] + ", " + holderStatuses[1] + ", " + holderStatuses[2] +
                '}';
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
    /*
    public void goToAngle(double target) {
        double error = getError(target);

        // DERIVATIVE (NOT USED)
        double derivative = (error - lastError) / pidTimer.seconds();

        Derivative filtering to reduce oscillations
        filteredDerivative = alpha * derivative + (1-alpha) * filteredDerivative;

        double output = (kP * error) + (-kD * filteredDerivative);


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
    */
    public void goToAngle(double target) {
        double error = getError(target);

        // Feedforward to overcome static friction (kS = 0.095)
        double ff = kS * Math.signum(error);

        double output = (kP * error) + ff;

        // Clipping output
        output = Range.clip(output, -0.4, 0.4);

        spindexerServo.setPower(output);
    }

    public double getError(double target) {
        return AngleUnit.normalizeDegrees(target - getWrappedAngle());
    }

    public double getContinuousAngle() {
        return spindexerEncoder.getContinuousAngle();
    }

    public double getWrappedAngle() {
        return spindexerEncoder.getWrappedAngle();
    }

    public void rebaseContinuousAngle() {
        spindexerEncoder.rebaseContinuousAngle();
    }

    // Returns the sequence of shooting angles starting from the closest (for shooting)
    public int[] getLaunchAngleSequence() {
        int current = (int) getWrappedAngle();

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
    public float[] getHSVRev() {
        int r = colorSensorV3.red();
        int g = colorSensorV3.green();
        int b = colorSensorV3.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);

        return hsv;
    }
}