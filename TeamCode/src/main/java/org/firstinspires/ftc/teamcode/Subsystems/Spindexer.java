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
    private int[] launchHolderAngles = {233, 352, 106};
    private int[] intakePositions = {46, 172, 287};
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

    /** Intake Code (CLEAN UP UNNECESSARY METHODS)*/
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

    /** PID Code */
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

    public double getWrappedAngle() {
        return spindexerEncoder.getWrappedAngle();
    }

    public boolean reachedTarget(double current, int target) {
        return Math.abs(current - target) < 3;
    }

    /** Color sensing intake logic */
    public void setHolderStatus(int index, HolderStatus status) {
        holderStatuses[index] = status;
    }

    public HolderStatus[] getHolderStatus() {
        return holderStatuses;
    }

    public void resetHolderStatuses() {
        holderStatuses[0] = HolderStatus.NONE; holderStatuses[1] = HolderStatus.NONE; holderStatuses[2] = HolderStatus.NONE;
    }

    public int[] getLaunchPositions() {
        return launchHolderAngles;
    }

    public int[] getLaunchPositionsDyanmic() {
        int[] launchPositions = {999, 999, 999};

        if (holderStatuses[0] != HolderStatus.NONE) {
            launchPositions[0] = launchHolderAngles[0];
        }
        if (holderStatuses[1] != HolderStatus.NONE) {
            launchPositions[1] = launchHolderAngles[1];
        }
        if (holderStatuses[2] != HolderStatus.NONE) {
            launchPositions[2] = launchHolderAngles[2];
        }

        return launchPositions;
    }

    public int[] getIntakePositions() {
        return intakePositions;
    }

    /** Color sensor code */
    public float[] getHSVRev() {
        int r = colorSensorV3.red();
        int g = colorSensorV3.green();
        int b = colorSensorV3.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);

        return hsv;
    }
}
