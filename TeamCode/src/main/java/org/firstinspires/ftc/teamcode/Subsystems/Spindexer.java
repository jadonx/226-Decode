package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.tanh;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorPIDControlLoopCoefficientsCommand;
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
import org.firstinspires.ftc.teamcode.Subsystems.Supporters.MotionProfiler;

@Config
public class Spindexer {
    private CRServo spindexerServo;
    private SpindexerEncoder spindexerEncoder;

    private double currentAngle;
    private double profileStartAngle;
    private double targetAngle = 0;
    public static double kP = 0.002, kV = 0;
    public static double maxVel = 0, maxAcc = 0;
    public double reference;
    private MotionProfiler profiler;
    private ElapsedTime profilerTimer = new ElapsedTime();

    public enum SpindexerMode {
        INTAKE_MODE,
        LAUNCH_MODE
    }
    SpindexerMode spindexerMode;
    private double spindexerSpeed = 0.4;
    private boolean isUnjamming;
    private boolean isUnjammingCW;
    private boolean startMotionProfile;

    private double lastAngle = 0;
    private double unwrappedAngle = 0;

    private int[] intakePositions = {114, 355, 233};
    public enum HolderStatus { NONE, GREEN, PURPLE }
    private HolderStatus[] holderStatuses = {HolderStatus.NONE, HolderStatus.NONE, HolderStatus.NONE};
    private HolderStatus[] motifPattern = new Spindexer.HolderStatus[] {Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.GREEN};

    public Spindexer(HardwareMap hardwareMap) {
        spindexerServo = hardwareMap.get(CRServo.class, Constants.HMServospinDexer);
        spindexerServo.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexerEncoder = hardwareMap.get(SpindexerEncoder.class, Constants.HMSpindexerEncoder);
        spindexerMode = SpindexerMode.INTAKE_MODE;
    }

    public void update() {
        updateAngles();

        if (isUnjamming) {
            if (isUnjammingCW) {
                spindexerServo.setPower(-0.3);
            }
            else {
                spindexerServo.setPower(0.3);
            }
            startMotionProfile = true;
        }
        else if (spindexerMode == SpindexerMode.INTAKE_MODE) {
            if (startMotionProfile) {
                startMotionProfile = false;
                startProfile(targetAngle, maxVel, maxAcc);
            }

            updateIntakeMode();
        }
        else if (spindexerMode == SpindexerMode.LAUNCH_MODE) {
            updateLaunchMode();
            startMotionProfile = true;
        }
    }

    public void updateAngles() {
        currentAngle = spindexerEncoder.getWrappedAngle();
        updateUnwrappedAngle();
    }

    private void startProfile(double targetAngle, double maxVel, double maxAcc) {
        profileStartAngle = unwrappedAngle;
        double targetUnwrapped = unwrappedAngle + AngleUnit.normalizeDegrees(targetAngle - currentAngle);
        double distance = targetUnwrapped - unwrappedAngle;
        profiler = new MotionProfiler(distance, maxVel, maxAcc);
        profilerTimer.reset();
    }

    private void updateIntakeMode() {
        if (profiler == null) return;

        double t = profilerTimer.seconds();
        reference = profiler.getReference(t);

        double desiredAngle = profileStartAngle + reference;
        double error = desiredAngle - unwrappedAngle; // calculateError(desiredAngle);

        double power = kP * error;

        // Velocity control
        double vRef = profiler.getVelocity(t);
        power += kV * vRef;

        spindexerServo.setPower(power);
    }

    private void updateLaunchMode() {
        if (getUnwrappedAngle() < targetAngle) {
            spindexerServo.setPower(spindexerSpeed);
        }
        else {
            spindexerServo.setPower(0);
        }
    }

    public boolean atTargetAngle(double threshold) {
        if (spindexerMode == SpindexerMode.INTAKE_MODE) {
            double error = Math.abs(targetAngle - getWrappedAngle());
            error = Math.min(error, 360 - error);

            return error < threshold;
        }
        else if (spindexerMode == SpindexerMode.LAUNCH_MODE) {
            return getUnwrappedAngle() > targetAngle;
        }

        return false;
    }

    private double calculateError(double targetAngle) {
        if (spindexerMode == SpindexerMode.INTAKE_MODE) {
            return AngleUnit.normalizeDegrees(targetAngle - currentAngle);
        }
        else {
            return 0;
        }
    }

    private void updateUnwrappedAngle() {
        double delta = AngleUnit.normalizeDegrees(currentAngle - lastAngle);
        unwrappedAngle += delta;
        lastAngle = currentAngle;
    }

    /** AUTO SORTING METHOD */
    public double getSortedPosition() {
        // Determine the green ball in motif pattern to get offset
        int holderOffset;

        if (motifPattern[0] == HolderStatus.GREEN) {
            holderOffset = 0;
        }
        else if (motifPattern[1] == HolderStatus.GREEN) {
            holderOffset = 1;
        }
        else {
            holderOffset = 2;
        }

        // Determine the green ball in current holders and return desired position
        if (holderStatuses[0] == HolderStatus.GREEN) {
            return intakePositions[(2 + holderOffset) % 3];
        }
        else if (holderStatuses[1] == HolderStatus.GREEN) {
            return intakePositions[holderOffset % 3];
        }
        else if (holderStatuses[2] == HolderStatus.GREEN) {
            return intakePositions[(1 + holderOffset) % 3];
        }
        else {
            return intakePositions[2];
        }
    }

    public void setMotifPattern(HolderStatus h1, HolderStatus h2, HolderStatus h3) {
        motifPattern[0] = h1;
        motifPattern[1] = h2;
        motifPattern[2] = h3;
    }

    public HolderStatus[] getMotifPattern() {
        return motifPattern;
    }

    /** SETTER AND GETTER METHODS */
    public void setMode(SpindexerMode newMode) {
        if (spindexerMode != newMode) {
            if (newMode == SpindexerMode.LAUNCH_MODE) {
                targetAngle = getUnwrappedAngle();
                targetAngle += 400;
            } else {
                targetAngle = currentAngle;
            }
            spindexerMode = newMode;
        }
    }

    public void setSpeed(double speed) {
        this.spindexerSpeed = speed;
    }

    public SpindexerMode getMode() {
        return spindexerMode;
    }

    public void setTargetAngle(double targetAngle) {
        if (this.targetAngle != targetAngle && spindexerMode == SpindexerMode.INTAKE_MODE) {
            // New target angle, start a new motion profile
            startMotionProfile = true;
        }
        this.targetAngle = targetAngle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getUnwrappedAngle() {
        return unwrappedAngle;
    }

    public double getWrappedAngle() {
        return currentAngle;
    }

    public void setPower(double power) {
        spindexerServo.setPower(power);
    }

    public void toggleUnjam(boolean isUnjammingCW) {
        isUnjamming = !isUnjamming;
        this.isUnjammingCW = isUnjammingCW;
    }

    /** Color sensing intake logic */
    public void setHolderStatus(int index, HolderStatus status) {
        holderStatuses[index] = status;
    }

    public void resetHolderStatuses() {
        holderStatuses[0] = HolderStatus.NONE; holderStatuses[1] = HolderStatus.NONE; holderStatuses[2] = HolderStatus.NONE;
    }

    public HolderStatus getHolderStatus(int index) {
        return holderStatuses[index];
    }

    public int[] getIntakePositions() {
        return intakePositions;
    }
}