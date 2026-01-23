package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Actions.AutonomousActions;
import org.firstinspires.ftc.teamcode.Constants;

@Config
public class Turret {
    //Hardware
    private CRServo turretLeft, turretRight;
    private IMU turretEncoder;

    public static double kP = 0.0065; // Proportional gain on position error
    public static double kI = 0.0; // Integral gain on position error
    public static double kD = 0.0006; // Derivative gain on position error
    public static double kF = 0.05; // Feedforward to overcome static friction

    private double currentAngle;
    private double targetAngle = 0;
    private double error;

    double lastError = 0;
    long lastTime = System.nanoTime();

    public enum TurretMode {
        PINPOINT,
        LIMELIGHT
    }
    private TurretMode turretMode = TurretMode.PINPOINT;

    public Turret(HardwareMap hardwareMap) {
        turretRight = hardwareMap.get(CRServo.class, Constants.HMServoTurretRight);
        turretLeft = hardwareMap.get(CRServo.class, Constants.HMServoTurretLeft);

        turretRight.setDirection(DcMotorSimple.Direction.REVERSE);
        turretLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        turretEncoder = hardwareMap.get(IMU.class, Constants.HMTurretEncoder);

        IMU.Parameters turretParameters = new IMU.Parameters(new RevHubOrientationOnRobot( RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        turretEncoder.initialize(turretParameters);

        // turretEncoder.resetYaw();
    }

    public void update() {
        currentAngle = turretEncoder.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (turretMode == TurretMode.PINPOINT) {
            error = wrapDegrees(targetAngle - currentAngle);
        }

        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastTime) / 1.0e9;
        double derivative = 0;
        if (deltaTime > 0) {
            derivative = (error - lastError) / deltaTime;
        }
        double pTerm = kP * error;
        double dTerm = kD * derivative;
        double fTerm = Math.signum(error) * kF; // Static friction boooost!!!!
        double power = pTerm + dTerm + fTerm;

        power = Math.max(Math.min(power, 1), -1);

        if (Math.abs(error) < 0.5) {
            setPower(0);
        } else {
            setPower(power);
        }

        lastError = error;
        lastTime = currentTime;
    }

    public void setTarget(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public double getTarget() {
        return targetAngle;
    }

    public boolean atTargetAngle() {
        double error = wrapDegrees(targetAngle - currentAngle);
        return (Math.abs(error) < 0.5);
    }

    public double getError() {
        return error;
    }

    public void setLimelightError(double error) {
        this.error = error;
    }

    public void setMode(TurretMode mode) {
        turretMode = mode;
    }

    public TurretMode getMode() {
        return turretMode;
    }

    public double getTurretAngle() {
        return currentAngle;
    }

    private double wrapDegrees(double angle) {
        angle %= 360.0;
        if (angle <= -180) angle += 360;
        if (angle > 180) angle -= 360;
        return angle;
    }

    public void resetTurretIMU() {
        turretEncoder.resetYaw();
    }

    public void setPower(double power) {
        turretLeft.setPower(-power);
        turretRight.setPower(-power);
    }

    public double wrapDegRobot(double a) {
        a = (a + 180.0) % 360.0;
        if (a < 0) a += 360.0;
        return a - 180.0;
    }

    public double deltaDeg(double to, double from) {
        return wrapDegRobot(to - from);
    }

    public double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}