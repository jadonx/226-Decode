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

    public static double kP = -0.015; // Proportional gain on position error
    public static double kI = 0.0; // Integral gain on position error
    public static double kD = -0.0009; // Derivative gain on position error
    public static double kF = -0.1; // Feedforward to overcome static friction
    public static double tolerance = 4;
    private double power;

    private double currentAngle;
    public static double targetAngle = 0;
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
        double fTerm = kF * Math.signum(error); // Static friction boooost!!!!
        power = (Math.abs(error) > tolerance) ? (pTerm + dTerm + fTerm) : (kF * 0.75) * Math.signum(error);


        power = Math.max(Math.min(power, 1), -1);

        turretRight.setPower(power);

        lastError = error;
        lastTime = currentTime;
    }

    public double getPower() {
        return power;
    }

    public void resetIMU() {
        turretEncoder.resetYaw();
    }

    public void setTarget(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    // Holds the current angle relative to the bot, the turret won't move if the bot is rotated
    public void holdCurrentAngle() {
        setPower(0);
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