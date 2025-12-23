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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class Spindexer {
    // HARDWARE VARIABLES
    private CRServo spindexerServo;
    private SpindexerEncoder spindexerEncoder;

    // PID VARIABLES
    private double kP = 0.002, kD = 0, kS = 0.0575;
    private int slowingThreshold = 0;
    private double slowingMultiplier = 0;
    private double lastError = 0;
    private ElapsedTime pidTimer;

    // SPINDEXER ANGLE VALUES AND HOLDER STATUSES
    private int[] launchHolderAngles = {50, 289, 175};
    private int[] intakePositions = {235, 112, 1};
    public enum HolderStatus { NONE, GREEN, PURPLE }
    public HolderStatus[] holderStatuses = {HolderStatus.NONE, HolderStatus.NONE, HolderStatus.NONE};

    // COLOR SENSOR VARIABLES
    private RevColorSensorV3 colorSensorV3;

    public Spindexer(HardwareMap hardwareMap) {
        spindexerServo = hardwareMap.get(CRServo.class, Constants.HMServospinDexer);
        spindexerServo.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexerEncoder = hardwareMap.get(SpindexerEncoder.class, Constants.HMSpindexerEncoder);
        pidTimer = new ElapsedTime();
        colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, Constants.HMFrontColorSensor);
    }

    public void goToAngle(double target) {
        double error = getError(target);

        double dt = pidTimer.seconds();
        pidTimer.reset();

        if (dt < 0.001) dt = 0.001;

        double derivative = (error - lastError) / dt;

        // Feedforward to overcome static friction (kS = 0.095)
        double ff = kS * Math.signum(error);

        double output = (kP * error) + (kD * derivative) + ff;

        if (Math.abs(error) < 2) {
            output = 0;
        }
        else if (Math.abs(error) < slowingThreshold) {
            output *= slowingMultiplier;
        }

        // Clipping output
        output = Range.clip(output, -0.4, 0.4);

        spindexerServo.setPower(output);

        lastError = error;
    }

    public double getError(double target) {
        return AngleUnit.normalizeDegrees(target - getWrappedAngle());
    }

    public double getWrappedAngle() {
        return spindexerEncoder.getWrappedAngle();
    }

    public boolean reachedTarget(double current, int target) {
        return Math.abs(current - target) < 5.5;
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

    public double getColorDistance() {
        return colorSensorV3.getDistance(DistanceUnit.INCH);
    }
}